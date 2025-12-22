import socket
import struct
import threading
import time
import signal
from dataclasses import dataclass
from enum import IntEnum
from typing import Optional, Tuple, List


# =========================
# Protocol definitions
# =========================

S3_MAGIC = 0x5333

class MsgType(IntEnum):
  CMD             = 1
  ACK             = 2
  POSE            = 3
  ODOMETRY        = 4
  SCAN            = 5
  IMU             = 6
  MATCHING_STATUS = 7
  MAP_POINTS      = 8
  MAP_DATA        = 9
  ERROR           = 10

class StreamId(IntEnum):
  CONTROL   = 0
  TELEMETRY = 1
  DEBUG     = 2
  LOGGING   = 3


# Header is packed in C++ with network conversion:
# magic: htons, stamp: htonll, len: htonl, seq: htonl
# => on-wire is big-endian for those fields.
HEADER_FMT = "!H Q I B B I"  # 2+8+4+1+1+4 = 20 bytes
HEADER_SIZE = struct.calcsize(HEADER_FMT)
CRC_SIZE = 2


# Payload structs (raw memcpy in C++ => assume little-endian)
POSE_FMT = "<fff"  # x,y,yaw

ODOM_FMT = "<ffffff"  # x,y,yaw,vx,vy,wz

IMU_FMT = "<ffffff"   # ax,ay,az,gx,gy,gz

MATCHING_STATUS_FMT = "<B i f f f"
# uint8 flag; int num_matches; float rmse; float inlier_ratio; float eigenvalue_ratio

# ScanPacket:
# 7 floats + 400 ranges + 400 intensities
SCAN_FMT = "<" + "f"*7 + "f"*400 + "f"*400

POINTS_HEADER_FMT = "<I"  # uint32 num_points
POINT_XY_FMT = "<ff"

# MapDataPacketHeader in C++:
# int width,height; float resolution,originX,originY,originYaw; int negate;
# float occupiedThresh, freeThresh; uint32 dataSize
MAP_DATA_HEADER_FMT = "<ii f f f f i f f I"
MAP_DATA_HEADER_SIZE = struct.calcsize(MAP_DATA_HEADER_FMT)


def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
  crc = init
  for b in data:
    crc ^= (b << 8) & 0xFFFF
    for _ in range(8):
      if crc & 0x8000:
        crc = ((crc << 1) ^ 0x1021) & 0xFFFF
      else:
        crc = (crc << 1) & 0xFFFF
  return crc & 0xFFFF


def recv_exact(sock: socket.socket, n: int) -> Optional[bytes]:
  buf = bytearray()
  while len(buf) < n:
    chunk = sock.recv(n - len(buf))
    if not chunk:
      return None
    buf.extend(chunk)
  return bytes(buf)


def send_all(sock: socket.socket, data: bytes) -> bool:
  view = memoryview(data)
  sent = 0
  while sent < len(data):
    n = sock.send(view[sent:])
    if n <= 0:
      return False
    sent += n
  return True


@dataclass
class Header:
  magic: int
  stamp: int
  length: int
  msg_type: int
  stream_id: int
  seq: int


def pack_header(stamp_ns: int, msg_type: int, stream_id: int, seq: int, payload_len: int) -> bytes:
  total = HEADER_SIZE + payload_len + CRC_SIZE
  return struct.pack(
    HEADER_FMT,
    S3_MAGIC,
    stamp_ns,
    total,
    int(msg_type) & 0xFF,
    int(stream_id) & 0xFF,
    seq & 0xFFFFFFFF,
  )


def unpack_header(b: bytes) -> Header:
  magic, stamp, length, msg_type, stream_id, seq = struct.unpack(HEADER_FMT, b)
  return Header(magic, stamp, length, msg_type, stream_id, seq)


def send_frame(sock: socket.socket, msg_type: MsgType, stream_id: StreamId, seq_ref: List[int],
               payload: bytes = b"", stamp_ns: Optional[int] = None) -> bool:
  if stamp_ns is None:
    stamp_ns = time.monotonic_ns()

  seq = seq_ref[0]
  seq_ref[0] += 1

  h = pack_header(stamp_ns, msg_type, stream_id, seq, len(payload))
  crc = crc16_ccitt(h + payload)
  frame = h + payload + struct.pack("!H", crc)  # CRC is network order like htons()

  return send_all(sock, frame)


def recv_frame(sock: socket.socket) -> Optional[Tuple[Header, bytes]]:
  hb = recv_exact(sock, HEADER_SIZE)
  if hb is None:
    return None
  h = unpack_header(hb)

  if h.magic != S3_MAGIC:
    return None

  total = h.length
  if total < HEADER_SIZE + CRC_SIZE:
    return None

  paylen = total - HEADER_SIZE - CRC_SIZE
  payload = b""
  if paylen > 0:
    payload = recv_exact(sock, paylen)
    if payload is None:
      return None

  crc_b = recv_exact(sock, CRC_SIZE)
  if crc_b is None:
    return None
  (crc_recv,) = struct.unpack("!H", crc_b)

  crc_calc = crc16_ccitt(hb + payload)
  if crc_calc != crc_recv:
    return None

  return h, payload


# =========================
# S3Node (Python)
# =========================

class S3Node:
  def __init__(self, ip: str, port: int = 7000):
    self._ip = ip
    self._port = port

    self._sock_lock = threading.Lock()
    self._sock: Optional[socket.socket] = None

    self._run = threading.Event()
    self._run.set()

    self._mutex = threading.Lock()

    self._got_map_data = True
    self._map_data_header = None
    self._map_data = b""

    self._got_map_points = True
    self._map_points: List[Tuple[float, float]] = []

    self._latest_streaming_stamp = 0

    self._latest_odom_stamp = 0
    self._latest_odom = None

    self._latest_scan_stamp = 0
    self._latest_scan = None

    self._latest_imu_stamp = 0
    self._latest_imu = None

    self._latest_status_stamp = 0
    self._latest_status = None

    self._send_cmd_flag = threading.Event()
    self._cmd = ""

    self._reset_pose_flag = threading.Event()
    self._reset_pose = (0.0, 0.0, 0.0)

    self._client_thread = threading.Thread(target=self._client_loop, daemon=True)
    self._client_thread.start()

  def close(self):
    self._run.clear()
    self._shutdown_socket()
    self._client_thread.join(timeout=2.0)

  # --- public API (same spirit as C++) ---

  def request_map_data(self):
    with self._mutex:
      self._got_map_data = False

  def request_map_points(self):
    with self._mutex:
      self._got_map_points = False

  def get_map_data(self):
    with self._mutex:
      if self._map_data_header is not None and self._map_data:
        hdr = self._map_data_header
        data = self._map_data
        self._map_data_header = None
        self._map_data = b""
        return hdr, data
      return None

  def get_map_points(self):
    with self._mutex:
      if self._map_points:
        pts = self._map_points
        self._map_points = []
        return pts
      return None

  def get_latest_odom(self, latest_stamp: int):
    with self._mutex:
      if self._latest_odom is not None and self._latest_odom_stamp > latest_stamp:
        return self._latest_odom_stamp, self._latest_odom
      return None

  def get_latest_scan(self, latest_stamp: int):
    with self._mutex:
      if self._latest_scan is not None and self._latest_scan_stamp > latest_stamp:
        return self._latest_scan_stamp, self._latest_scan
      return None

  def get_latest_imu(self, latest_stamp: int):
    with self._mutex:
      if self._latest_imu is not None and self._latest_imu_stamp > latest_stamp:
        return self._latest_imu_stamp, self._latest_imu
      return None

  def get_latest_matching_status(self, latest_stamp: int):
    with self._mutex:
      if self._latest_status is not None and self._latest_status_stamp > latest_stamp:
        return self._latest_status_stamp, self._latest_status
      return None

  def send_cmd(self, cmd: str):
    with self._mutex:
      self._cmd = cmd
    self._send_cmd_flag.set()

  def reset_pose(self, x: float, y: float, yaw: float):
    with self._mutex:
      self._reset_pose = (x, y, yaw)
    self._reset_pose_flag.set()

  # --- internals ---

  def _connect(self):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    s.connect((self._ip, self._port))
    s.settimeout(None)
    with self._sock_lock:
      self._sock = s

  def _shutdown_socket(self):
    with self._sock_lock:
      s = self._sock
      self._sock = None
    if s is not None:
      try:
        s.shutdown(socket.SHUT_RDWR)
      except OSError:
        pass
      try:
        s.close()
      except OSError:
        pass

  def _get_sock(self) -> Optional[socket.socket]:
    with self._sock_lock:
      return self._sock

  def _client_loop(self):
    while self._run.is_set():
      try:
        self._connect()
      except OSError:
        time.sleep(0.5)
        continue

      rx_ok = threading.Event()
      rx_ok.set()

      rx = threading.Thread(target=self._rx_loop, args=(rx_ok,), daemon=True)
      tx = threading.Thread(target=self._tx_loop, args=(rx_ok,), daemon=True)
      rx.start()
      tx.start()

      rx.join()
      rx_ok.clear()
      tx.join()

      if self._run.is_set():
        self._shutdown_socket()
        time.sleep(0.2)

  def _rx_loop(self, rx_ok: threading.Event):
    while rx_ok.is_set() and self._run.is_set():
      s = self._get_sock()
      if s is None:
        break

      fr = recv_frame(s)
      if fr is None:
        break

      h, pl = fr

      stamp = h.stamp
      msg_type = MsgType(h.msg_type)
      stream_id = StreamId(h.stream_id)

      if stream_id == StreamId.TELEMETRY:
        with self._mutex:
          self._latest_streaming_stamp = stamp

      if msg_type == MsgType.ERROR:
        break

      if msg_type == MsgType.ACK:
        continue

      if msg_type == MsgType.MAP_DATA:
        parsed = self._parse_map_data(pl)
        if parsed is None:
          break
        header, data = parsed
        with self._mutex:
          self._got_map_data = True
          self._map_data_header = header
          self._map_data = data

      elif msg_type == MsgType.MAP_POINTS:
        pts = self._parse_map_points(pl)
        if pts is None:
          break
        with self._mutex:
          self._got_map_points = True
          self._map_points = pts

      elif msg_type == MsgType.ODOMETRY:
        odom = self._parse_fixed(pl, ODOM_FMT)
        if odom is None:
          break
        with self._mutex:
          self._latest_odom_stamp = stamp
          self._latest_odom = odom

      elif msg_type == MsgType.SCAN:
        scan = self._parse_fixed(pl, SCAN_FMT)
        if scan is None:
          break
        with self._mutex:
          self._latest_scan_stamp = stamp
          self._latest_scan = scan

      elif msg_type == MsgType.IMU:
        imu = self._parse_fixed(pl, IMU_FMT)
        if imu is None:
          break
        with self._mutex:
          self._latest_imu_stamp = stamp
          self._latest_imu = imu

      elif msg_type == MsgType.MATCHING_STATUS:
        st = self._parse_fixed(pl, MATCHING_STATUS_FMT)
        if st is None:
          break
        with self._mutex:
          self._latest_status_stamp = stamp
          self._latest_status = st

    rx_ok.clear()

  def _tx_loop(self, rx_ok: threading.Event):
    seq_ref = [0]

    s = self._get_sock()
    if s is None:
      rx_ok.clear()
      return

    send_frame(s, MsgType.CMD, StreamId.CONTROL, seq_ref, b"start_streaming")

    while rx_ok.is_set() and self._run.is_set():
      time.sleep(1.0)

      s = self._get_sock()
      if s is None:
        break

      with self._mutex:
        need_map = not self._got_map_data
        need_pts = not self._got_map_points

      if need_map:
        send_frame(s, MsgType.CMD, StreamId.CONTROL, seq_ref, b"get_map_data")

      if need_pts:
        send_frame(s, MsgType.CMD, StreamId.CONTROL, seq_ref, b"get_map_points")

      if self._send_cmd_flag.is_set():
        with self._mutex:
          cmd = self._cmd.encode("utf-8")
        ok = send_frame(s, MsgType.CMD, StreamId.CONTROL, seq_ref, cmd)
        self._send_cmd_flag.clear()
        if not ok:
          rx_ok.clear()
          break

      if self._reset_pose_flag.is_set():
        with self._mutex:
          x, y, yaw = self._reset_pose
        payload = struct.pack(POSE_FMT, x, y, yaw)
        stamp_ns = time.monotonic_ns()
        ok = send_frame(s, MsgType.POSE, StreamId.TELEMETRY, seq_ref, payload, stamp_ns=stamp_ns)
        self._reset_pose_flag.clear()
        if not ok:
          rx_ok.clear()
          break

    s = self._get_sock()
    if s is not None:
      send_frame(s, MsgType.CMD, StreamId.CONTROL, seq_ref, b"stop_streaming")
      time.sleep(1.0)

  def _parse_fixed(self, pl: bytes, fmt: str):
    sz = struct.calcsize(fmt)
    if len(pl) != sz:
      return None
    return struct.unpack(fmt, pl)

  def _parse_map_data(self, pl: bytes):
    if len(pl) < MAP_DATA_HEADER_SIZE:
      return None
    header_vals = struct.unpack_from(MAP_DATA_HEADER_FMT, pl, 0)
    # unpack fields for readability
    header = {
      "width": header_vals[0],
      "height": header_vals[1],
      "resolution": header_vals[2],
      "originX": header_vals[3],
      "originY": header_vals[4],
      "originYaw": header_vals[5],
      "negate": header_vals[6],
      "occupiedThresh": header_vals[7],
      "freeThresh": header_vals[8],
      "dataSize": header_vals[9],
    }
    expected = MAP_DATA_HEADER_SIZE + header["dataSize"]
    if len(pl) != expected:
      return None
    data = pl[MAP_DATA_HEADER_SIZE:expected]  # bytes of signed char
    return header, data

  def _parse_map_points(self, pl: bytes):
    if len(pl) < struct.calcsize(POINTS_HEADER_FMT):
      return None
    (num_points,) = struct.unpack_from(POINTS_HEADER_FMT, pl, 0)
    pts_off = struct.calcsize(POINTS_HEADER_FMT)
    pts_sz = num_points * struct.calcsize(POINT_XY_FMT)
    if len(pl) != pts_off + pts_sz:
      return None
    pts = []
    off = pts_off
    for _ in range(num_points):
      x, y = struct.unpack_from(POINT_XY_FMT, pl, off)
      pts.append((x, y))
      off += struct.calcsize(POINT_XY_FMT)
    return pts