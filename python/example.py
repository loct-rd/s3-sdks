import sys
import time
import math
import threading
import signal
from s3_node import S3Node

g_signal = threading.Event()

def _handler(signum, frame):
  g_signal.set()

def main():
  ip = "192.168.11.100"
  request_map = False
  request_points = False

  args = sys.argv[1:]
  for a in args:
    if a == "--map":
      request_map = True
    elif a == "--points":
      request_points = True
    elif not a.startswith("-"):
      ip = a
    else:
      print("Unknown option:", a)

  print("Using S3 IP address :", ip)
  print("Request map data    :", "yes" if request_map else "no")
  print("Request map points  :", "yes" if request_points else "no")

  signal.signal(signal.SIGINT, _handler)
  signal.signal(signal.SIGTERM, _handler)

  node = S3Node(ip)

  if request_map:
    node.request_map_data()
    t0 = time.monotonic()
    while not g_signal.is_set():
      got = node.get_map_data()
      if got is not None:
        hdr, data = got
        print("Got map data.")
        print("width          =", hdr["width"])
        print("height         =", hdr["height"])
        print("resolution     =", hdr["resolution"])
        print("origin         = [", hdr["originX"], ",", hdr["originY"], ",", hdr["originYaw"], "]")
        print("negate         =", "yes" if hdr["negate"] else "no")
        print("occupiedThresh =", hdr["occupiedThresh"])
        print("freeThresh     =", hdr["freeThresh"])
        print("mapDataSize    =", len(data))
        with open("map_data.raw", "wb") as f:
          f.write(data)
        break
      if time.monotonic() - t0 > 5.0:
        print("Timeout waiting map data.")
        break
      time.sleep(0.1)

  if request_points:
    node.request_map_points()
    t0 = time.monotonic()
    while not g_signal.is_set():
      pts = node.get_map_points()
      if pts is not None and len(pts) > 0:
        print("Got map points.")
        print("mapPointsSize =", len(pts))
        with open("map_points.txt", "w") as f:
          for x, y in pts:
            f.write(f"{x} {y}\n")
        break
      if time.monotonic() - t0 > 5.0:
        print("Timeout waiting map points.")
        break
      time.sleep(0.1)

  latest_odom_stamp = 0
  latest_scan_stamp = 0
  latest_imu_stamp = 0
  latest_status_stamp = 0
  kPi = math.pi

  try:
    while not g_signal.is_set():
      time.sleep(0.01)

      od = node.get_latest_odom(latest_odom_stamp)
      if od is not None:
        latest_odom_stamp, v = od
        x, y, yaw, vx, vy, wz = v
        print("Got new odom.")
        print(f"x   = {x} [m]")
        print(f"y   = {y} [m]")
        print(f"yaw = {yaw * 180.0 / kPi} [deg]")
        print(f"vx  = {vx} [m/s]")
        print(f"vy  = {vy} [m/s]")
        print(f"wz  = {wz * 180.0 / kPi} [deg/s]")

      sc = node.get_latest_scan(latest_scan_stamp)
      if sc is not None:
        latest_scan_stamp, v = sc
        print("Got new scan.")
        # v は (7 + 400 + 400) floats の tuple
        # angle_min = v[0], ...
        # ranges = v[7:7+400]
        # intensities = v[7+400:7+400+400]

      im = node.get_latest_imu(latest_imu_stamp)
      if im is not None:
        latest_imu_stamp, v = im
        ax, ay, az, gx, gy, gz = v
        print("Got new IMU.")
        print(f"ax = {ax} [m/s^2]")
        print(f"ay = {ay} [m/s^2]")
        print(f"az = {az} [m/s^2]")
        print(f"gx = {gx * 180.0 / kPi} [deg/s]")
        print(f"gy = {gy * 180.0 / kPi} [deg/s]")
        print(f"gz = {gz * 180.0 / kPi} [deg/s]")

      st = node.get_latest_matching_status(latest_status_stamp)
      if st is not None:
        latest_status_stamp, v = st
        flag, num_matches, rmse, inlier_ratio, eigen_ratio = v
        print("Got new matching status.")
        print("flag             :", int(flag))
        print("num_matches      :", num_matches)
        print("rmse             :", rmse, "[m]")
        print("inlier_ratio     :", inlier_ratio * 100.0, "[%]")
        print("eigenvalue_ratio :", eigen_ratio * 100.0, "[%]")

  finally:
    node.close()

if __name__ == "__main__":
  main()