/*
 * ========================================================== 
 * Project Name    : S3 project
 * File Name       : net_utils.h
 * Encoding        : ASCII (LF)
 * Creation Date   : 18, Nov, 2025
 * 
 * Copyright (C) 2025 LOCT Co., Ltd. All rights reserved.
 *  
 * This source code or any portion thereof must not be  
 * reproduced or used in any manner whatsoever.
 * ==========================================================
 */

#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>

#include <vector>
#include <cstdint>
#include <cstddef>
#include <cstdio>
#include <chrono>

namespace net {

constexpr uint16_t S3_MAGIC = 0x5333;

enum class MsgType : uint8_t {
  CMD             = 1,
  ACK             = 2,
  POSE            = 3,
  ODOMETRY        = 4,
  SCAN            = 5,
  IMU             = 6,
  MATCHING_STATUS = 7,
  MAP_POINTS      = 8,
  MAP_DATA        = 9,
  ERROR           = 10,
};

enum class StreamId : uint8_t {
  CONTROL   = 0,
  TELEMETRY = 1,
  DEBUG     = 2,
  LOGGING   = 3,
};

struct Header {
  uint16_t magic;      // net: always 0xA55A
  uint64_t stamp;      // net
  uint32_t len;        // net
  uint8_t  type;       // MsgType
  uint8_t  stream_id;  // StreamId
  uint32_t seq;        // net
} __attribute__((packed));

#pragma pack(push, 1)
struct PosePacket {
  float x, y, yaw;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct OdomPacket {
  float x, y, yaw;
  float vx, vy, wz;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct ScanPacket {
  float angleMin;
  float angleMax;
  float angleIncrement;
  float rangeMin;
  float rangeMax;
  float timeIncrement;
  float scanTime;
  float ranges[400];
  float intensities[400];
};
#pragma pack(pop)

#pragma pack(push, 1)
struct IMUPacket {
  float ax, ay, az;
  float gx, gy, gz;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct PointXY {
  float x, y;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct PointsPacketHeader {
  uint32_t num_points;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct MapData {
  signed char data;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct MapDataPacketHeader {
  int width;
  int height;
  float resolution;
  float originX;
  float originY;
  float originYaw;
  int negate;
  float occupiedThresh;
  float freeThresh;
  uint32_t dataSize;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct MatchingStatusPacket {
  uint8_t flag;
  int num_matches;
  float rmse;
  float inlier_ratio;
  float eigenvalue_ratio;
};
#pragma pack(pop)

static inline MsgType toMsgType(uint8_t v) {
  return static_cast<MsgType>(v);
}

static inline StreamId toStreamId(uint8_t v) {
  return static_cast<StreamId>(v);
}

static inline uint64_t htonll(uint64_t x) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
  return (((uint64_t)htonl(x & 0xFFFFFFFFULL)) << 32) | htonl(x >> 32);
#else
  return x;
#endif
}

static inline uint64_t ntohll(uint64_t x) {
  return htonll(x);
}

// CRC16-CCITT (0x1021), init = 0xFFFF, no xorout
static inline uint16_t crc16_ccitt(uint16_t crc, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc = (crc << 1);
      }
    }
  }
  return crc;
}

static inline ssize_t sendAll(int sock, const void* buf, size_t len) {
  const uint8_t* p = static_cast<const uint8_t*>(buf);
  size_t off = 0;
  while (off < len) {
    const ssize_t n = ::send(sock, p + off, len - off, 0);
    if (n <= 0) {
      return n;
    }
    off += (size_t)n;
  }
  return (ssize_t)off;
}

static inline ssize_t recvAll(int sock, void* buf, size_t len) {
  uint8_t* p = static_cast<uint8_t*>(buf);
  size_t off = 0;
  while (off < len) {
    const ssize_t n = ::recv(sock, p + off, len - off, 0);
    if (n <= 0) {
      return n;
    }
    off += (size_t)n;
  }
  return (ssize_t)off;
}

static inline bool sendFrame(int sock, MsgType type, StreamId stream_id,
                             uint32_t& seq_counter, const void* payload,
                             uint32_t payload_size)
{
  Header h{};
  h.magic = htons(S3_MAGIC);
  const uint64_t stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::steady_clock::now().time_since_epoch()).count();
  h.stamp = htonll(stamp);

  const uint32_t total = static_cast<uint32_t>(sizeof(Header)) +
                         payload_size + sizeof(uint16_t);  // + CRC16
  h.len = htonl(total);
  h.type = static_cast<uint8_t>(type);
  h.stream_id = static_cast<uint8_t>(stream_id);
  h.seq = htonl(seq_counter++);

  uint16_t crc = 0xFFFF;
  crc = crc16_ccitt(crc,
                    reinterpret_cast<const uint8_t*>(&h),
                    sizeof(h));
  if (payload_size > 0 && payload != nullptr) {
    crc = crc16_ccitt(crc,
                      static_cast<const uint8_t*>(payload),
                      payload_size);
  }
  uint16_t crc_net = htons(crc);

  if (sendAll(sock, &h, sizeof(h)) <= 0) {
    return false;
  }
  if (payload_size > 0 && sendAll(sock, payload, payload_size) <= 0) {
    return false;
  }
  if (sendAll(sock, &crc_net, sizeof(crc_net)) <= 0) {
    return false;
  }

  return true;
}

static inline bool sendFrame(int sock, uint64_t stamp, MsgType type,
                             StreamId stream_id, uint32_t& seq_counter,
                             const void* payload, uint32_t payload_size)
{
  Header h{};
  h.magic = htons(S3_MAGIC);
  h.stamp = htonll(stamp);

  const uint32_t total = static_cast<uint32_t>(sizeof(Header)) +
                         payload_size + sizeof(uint16_t);  // + CRC16
  h.len = htonl(total);
  h.type = static_cast<uint8_t>(type);
  h.stream_id = static_cast<uint8_t>(stream_id);
  h.seq = htonl(seq_counter++);

  uint16_t crc = 0xFFFF;
  crc = crc16_ccitt(crc,
                    reinterpret_cast<const uint8_t*>(&h),
                    sizeof(h));
  if (payload_size > 0 && payload != nullptr) {
    crc = crc16_ccitt(crc,
                      static_cast<const uint8_t*>(payload),
                      payload_size);
  }
  uint16_t crc_net = htons(crc);

  if (sendAll(sock, &h, sizeof(h)) <= 0) {
    return false;
  }
  if (payload_size > 0 && sendAll(sock, payload, payload_size) <= 0) {
    return false;
  }
  if (sendAll(sock, &crc_net, sizeof(crc_net)) <= 0) {
    return false;
  }

  return true;
}

static inline bool sendFrame(int sock, MsgType type, StreamId stream_id,
                             uint32_t& seq_counter, const std::string& payload)
{
  return sendFrame(sock, type, stream_id, seq_counter,
                   payload.data(), static_cast<uint32_t>(payload.size()));
}

static inline bool sendFrame(int sock, uint64_t stamp, MsgType type, StreamId stream_id,
                             uint32_t& seq_counter, const std::string& payload)
{
  return sendFrame(sock, stamp, type, stream_id, seq_counter,
                   payload.data(), static_cast<uint32_t>(payload.size()));
}

static inline bool recvFrame(int sock, Header& h, std::vector<uint8_t>& payload) {
  if (recvAll(sock, &h, sizeof(h)) <= 0) {
    return false;
  }

  if (ntohs(h.magic) != S3_MAGIC) {
    fprintf(stderr, "Invalid magic header: 0x%04X\n", ntohs(h.magic));
    return false;
  }

  const uint32_t total = ntohl(h.len);
  // Error if the size is less than header and CRC16
  if (total < sizeof(Header) + sizeof(uint16_t)) {
    return false;
  }

  const uint32_t paylen = total
                        - static_cast<uint32_t>(sizeof(Header))
                        - static_cast<uint32_t>(sizeof(uint16_t));

  payload.resize(paylen);

  // Read payload
  if (paylen > 0 && recvAll(sock, payload.data(), paylen) <= 0) {
    return false;
  }

  // Read CRC16
  uint16_t crc_net = 0;
  if (recvAll(sock, &crc_net, sizeof(crc_net)) <= 0) {
    return false;
  }
  const uint16_t crc_recv = ntohs(crc_net);

  // Check CRC16
  uint16_t crc = 0xFFFF;
  crc = crc16_ccitt(crc, reinterpret_cast<const uint8_t*>(&h), sizeof(h));
  if (paylen > 0) {
    crc = crc16_ccitt(crc, payload.data(), paylen);
  }
  if (crc != crc_recv) {
    fprintf(stderr, "CRC16 mismatch: recv=0x%04X calc=0x%04X\n", crc_recv, crc);
    return false;
  }

  return true;
}

} // namespace net