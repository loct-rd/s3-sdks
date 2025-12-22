/*
 * Copyright (c) 2025 LOCT Co., Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * --------------------------------------------------------------------
 *
 * This file is part of the Small SLAM Sensor (S3) SDK.
 *
 * It provides a minimal C++ example demonstrating how to:
 *  - connect to an S3 device via TCP/IP,
 *  - request map data and map points,
 *  - receive odometry, IMU, scan, and matching status streams,
 *  - and gracefully terminate using signal handling.
 */

#pragma once

#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>

#include <string>
#include <mutex>
#include <thread>
#include <atomic>
#include <cstring>
#include <iostream>
#include <vector>
#include <chrono>
#include <cstdint>
#include <csignal>
#include <cmath>
#include <chrono>

#include <net_utils.h>

class S3Node {
 public:
  S3Node(const std::string& s3IpAddress):
    s3IpAddress_(s3IpAddress)
  {
    setupSocket();
    clientRunning_.store(true);
    clientThread_ = std::thread(&S3Node::clientLoop, this);
  }

  ~S3Node() {
    clientRunning_.store(false);
    ::shutdown(sock_, SHUT_RDWR);
    ::close(sock_);
    if (clientThread_.joinable()) {
      clientThread_.join();
    }
  }

  void join() {
    clientRunning_.store(false);
  }

  void requestMapData() {
    gotMapData_ = false;
  }

  void requestMapPoints() {
    gotMapPoints_ = false;
  }

  bool getMapData(net::MapDataPacketHeader& header, std::vector<signed char>& data) {
    if (gotMapData_ && mapData_.size() > 0) {
      header = mapDataHeader_;
      data = mapData_;
      mapData_.clear();
      return true;
    } else {
      return false;
    }
  }

  bool getMapPoints(std::vector<net::PointXY>& points) {
    if (gotMapPoints_ && mapPoints_.size() > 0) {
      points = mapPoints_;
      mapPoints_.clear();
      return true;
    } else {
      return false;
    }
  }

  bool getLatestOdom(uint64_t& latestStamp, net::OdomPacket& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (latestOdomStamp_ > latestStamp) {
      latestStamp = latestOdomStamp_;
      out = latestOdom_;
      return true;
    } else {
      return false;
    }
  }

  bool getLatestScan(uint64_t& latestStamp, net::ScanPacket& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (latestScanStamp_ > latestStamp) {
      latestStamp = latestScanStamp_;
      out = latestScan_;
      return true;
    } else {
      return false;
    }
  }

  bool getLatestImu(uint64_t& latestStamp, net::IMUPacket& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (latestImuStamp_ > latestStamp) {
      latestStamp = latestImuStamp_;
      out = latestImu_;
      return true;
    } else {
      return false;
    }
  }

  bool getLatestMatchingStatus(uint64_t& latestStamp, net::MatchingStatusPacket& out) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (latestMatchingStatusStamp_ > latestStamp) {
      latestStamp = latestMatchingStatusStamp_;
      out = latestMatchingStatus_;
      return true;
    } else {
      return false;
    }
  }

  void sendCmd(const std::string& cmd) {
    std::lock_guard<std::mutex> lock(mutex_);
    cmd_ = cmd;
    sendCmd_.store(true);
  }

  void resetPose(float x, float y, float yaw) {
    std::lock_guard<std::mutex> lock(mutex_);
    resetPoseX_   = x;
    resetPoseY_   = y;
    resetPoseYaw_ = yaw;
    resetPose_.store(true);
  }

 private:
  void setupSocket() {
    // Ignore broken pipe error
    signal(SIGPIPE, SIG_IGN);

    sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(7000);
    ::inet_pton(AF_INET, s3IpAddress_.c_str(), &addr.sin_addr);
    if (connect(sock_, (sockaddr*)&addr, sizeof(addr)) < 0) {
      perror("connect");
      exit(1);
    }

    const int yes = 1;
    ::setsockopt(sock_, IPPROTO_TCP, TCP_NODELAY, &yes, sizeof(yes));
  }

  void cleanupSocket() {
    ::shutdown(sock_, SHUT_RDWR);
    ::close(sock_);
  }

  void clientLoop() {
    std::atomic<bool> run(true);

    while (clientRunning_.load()) {
      std::thread rx(&S3Node::rxLoop, this, sock_, std::ref(run));
      std::thread tx(&S3Node::txLoop, this, sock_, std::ref(run));
      rx.join();
      run.store(false);
      tx.join();

      if (clientRunning_.load()) {
        cleanupSocket();
        setupSocket();
        run.store(true);
        std::cout << "Restart client loop." << std::endl;
      }
    }
  }

  void rxLoop(
    int sock,
    std::atomic<bool>& run)
  {
    while (run.load() && clientRunning_.load()) {
      net::Header h{};
      std::vector<uint8_t> pl;
      if (!net::recvFrame(sock, h, pl)) {
        std::cerr << "Failed to receive data from S3." << std::endl;
        break;
      }

      // const uint32_t seq = ntohl(h.seq);
      const uint64_t stamp = net::ntohll(h.stamp);
      const net::MsgType type = net::toMsgType(h.type);
      const net::StreamId streamId = net::toStreamId(h.stream_id);
      if (streamId == net::StreamId::TELEMETRY) {
        std::lock_guard<std::mutex> lock(mutex_);
        latestStreamingStamp_ = stamp;
      }

      if (type == net::MsgType::ERROR) {
        std::cerr << "Got error message from S3." << std::endl;
        break;

      } else if (type == net::MsgType::ACK) {
        std::cout << "Got ACK." << std::endl;

      } else if (type == net::MsgType::MAP_DATA) {
        if (pl.size() == 0) {
          std::cerr << "Map data payload is empty." << std::endl;
          continue;
        }
        net::MapDataPacketHeader header;
        std::vector<signed char> data;
        if (!parseMapDataPayload(pl, header, data)) {
          std::cerr << "Failed to parse map data payload." << std::endl;
          break;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        gotMapData_ = true;
        mapDataHeader_ = header;
        mapData_ = data;

      } else if (type == net::MsgType::MAP_POINTS) {
        if (pl.size() == 0) {
          std::cerr << "Map points payload is empty." << std::endl;
          continue;
        }
        net::PointsPacketHeader header;
        std::vector<net::PointXY> points;
        if (!parseMapPointsPayload(pl, header, points)) {
          std::cerr << "Failed to parse map points payload." << std::endl;
          break;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        gotMapPoints_ = true;
        mapPoints_ = points;
        
      } else if (type == net::MsgType::ODOMETRY) {
        net::OdomPacket pkt;
        if (!parseOdomPayload(pl, pkt)) {
          std::cerr << "Failed to parse odometry payload." << std::endl;
          break;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        latestOdomStamp_ = stamp;
        latestOdom_ = pkt;

      } else if (type == net::MsgType::SCAN) {
        net::ScanPacket pkt;
        if (!parseScanPayload(pl, pkt)) {
          std::cerr << "Failed to scan payload." << std::endl;
          break;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        latestScanStamp_ = stamp;
        latestScan_ = pkt;

      } else if (type == net::MsgType::IMU) {
        net::IMUPacket pkt;
        if (!parseIMUPayload(pl, pkt)) {
          std::cerr << "Failed to parse IMU payload." << std::endl;
          break;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        latestImuStamp_ = stamp;
        latestImu_ = pkt;

      } else if (type == net::MsgType::MATCHING_STATUS) {
        net::MatchingStatusPacket pkt;
        if (!parseMatchingStatusPayload(pl, pkt)) {
          std::cerr << "Failed to parse matchig status payload." << std::endl;
          break;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        latestMatchingStatusStamp_ = stamp;
        latestMatchingStatus_ = pkt;

      }
    }

    run.store(false);
  }

  bool parseMapDataPayload(
    const std::vector<uint8_t>& pl,
    net::MapDataPacketHeader& header,
    std::vector<signed char>& data)
  {
    if (pl.size() < sizeof(net::MapDataPacketHeader)) {
      return false;
    }

    const uint8_t* ptr = pl.data();
    std::memcpy(&header, ptr, sizeof(net::MapDataPacketHeader));
    const std::size_t header_size = sizeof(net::MapDataPacketHeader);
    const std::size_t expected_size = header_size + header.dataSize;
    if (pl.size() != expected_size) {
      return false;
    }

    const uint8_t* dataPtr = ptr + header_size;
    data.resize(header.dataSize);
    std::memcpy(data.data(), dataPtr, header.dataSize);

    return true;
  }

  bool parseMapPointsPayload(
    const std::vector<uint8_t>& pl,
    net::PointsPacketHeader& header,
    std::vector<net::PointXY>& points)
  {
    const std::size_t header_size = sizeof(net::PointsPacketHeader);
    if (pl.size() < header_size) {
      return false;
    }

    const uint8_t* ptr = pl.data();
    std::memcpy(&header, ptr, header_size);

    const uint32_t num_points = header.num_points;
    const std::size_t points_size = static_cast<std::size_t>(num_points) * sizeof(net::PointXY);
    const std::size_t expected_size = header_size + points_size;
    if (pl.size() != expected_size) {
      return false;
    }

    points.resize(num_points);
    const uint8_t* points_ptr = ptr + header_size;
    std::memcpy(points.data(), points_ptr, points_size);

    return true;
  }

  bool parseOdomPayload(
    const std::vector<uint8_t>& pl,
    net::OdomPacket& pkt)
  {
    if (pl.size() != sizeof(net::OdomPacket)) {
      return false;
    }
    std::memcpy(&pkt, pl.data(), sizeof(net::OdomPacket));
    return true;
  }

  bool parseScanPayload(
    const std::vector<uint8_t>& pl,
    net::ScanPacket& pkt)
  {
    if (pl.size() != sizeof(net::ScanPacket)) {
      return false;
    }
    std::memcpy(&pkt, pl.data(), sizeof(net::ScanPacket));
    return true;
  }

  bool parseIMUPayload(
    const std::vector<uint8_t>& pl,
    net::IMUPacket& pkt)
  {
    if (pl.size() != sizeof(net::IMUPacket)) {
      return false;
    }
    std::memcpy(&pkt, pl.data(), sizeof(net::IMUPacket));
    return true;
  }

  bool parseMatchingStatusPayload(
    const std::vector<uint8_t>& pl,
    net::MatchingStatusPacket& pkt)
  {
    if (pl.size() != sizeof(net::MatchingStatusPacket)) {
      return false;
    }
    std::memcpy(&pkt, pl.data(), sizeof(net::MatchingStatusPacket));
    return true;
  }

  void txLoop(
    int sock,
    std::atomic<bool>& run)
  {
    using namespace std::chrono_literals;
    uint32_t seq = 0;

    net::sendFrame(sock, net::MsgType::CMD, net::StreamId::CONTROL, seq, "start_streaming");

    while (run.load() && clientRunning_.load()) {
      std::this_thread::sleep_for(1s);

      if (!gotMapData_) {
        net::sendFrame(sock, net::MsgType::CMD, net::StreamId::CONTROL, seq, "get_map_data");
      }

      if (!gotMapPoints_) {
        net::sendFrame(sock, net::MsgType::CMD, net::StreamId::CONTROL, seq, "get_map_points");
      }

      if (sendCmd_.load()) {
        net::sendFrame(sock, net::MsgType::CMD, net::StreamId::CONTROL, seq, cmd_);
        sendCmd_.store(false);
        std::cout << "S3 node sent " << cmd_ << " command." << std::endl;
      }

      if (resetPose_.load()) {
        net::PosePacket pkt;
        pkt.x = resetPoseX_;
        pkt.y = resetPoseY_;
        pkt.yaw = resetPoseYaw_;
        std::vector<uint8_t> payload;
        payload.resize(sizeof(net::PosePacket));
        size_t payloadSize = payload.size();
        std::memcpy(payload.data(), &pkt, sizeof(net::PosePacket));
        const uint64_t stamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
        if (!net::sendFrame(sock, stamp, net::MsgType::POSE,
          net::StreamId::TELEMETRY, seq, payload.data(), payloadSize))
        {
          std::cerr << "Failed to send reset pose." << std::endl;
          run = false;
          break;
        }
        resetPose_.store(false);
        std::cout << "Sent reset pose command." << std::endl;
      }
    }

    net::sendFrame(sock, net::MsgType::CMD, net::StreamId::CONTROL, seq, "stop_streaming");
    sleep(1);
    run.store(false);
  }

  int sock_;
  std::string s3IpAddress_;
  std::thread clientThread_;
  std::atomic<bool> clientRunning_{false};
  uint64_t latestStreamingStamp_;

  mutable std::mutex mutex_;

  bool gotMapData_{true};
  uint64_t mapDataStamp_{0};
  net::MapDataPacketHeader mapDataHeader_;
  std::vector<signed char> mapData_;

  bool gotMapPoints_{true};
  std::vector<net::PointXY> mapPoints_;

  uint64_t latestOdomStamp_{0};
  net::OdomPacket latestOdom_{};
  uint64_t latestScanStamp_{0};
  net::ScanPacket latestScan_{};
  uint64_t latestImuStamp_{0};
  net::IMUPacket latestImu_{};
  uint64_t latestMatchingStatusStamp_{0};
  net::MatchingStatusPacket latestMatchingStatus_{};

  std::atomic<bool> sendCmd_{false};
  std::string cmd_;

  std::atomic<bool> resetPose_{false};
  float resetPoseX_;
  float resetPoseY_;
  float resetPoseYaw_;
}; // class S3Node
