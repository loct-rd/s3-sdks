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

#include <iostream>
#include <fstream>

#include <s3_node.h>

std::atomic<bool> g_signal{false};

extern "C" void signalHandler(int) {
  g_signal.store(true);
}

int main(int argc, char** argv) {
  // Default settings
  std::string ipAddress = "192.168.11.100";
  bool requestMapData   = false;
  bool requestMapPoints = false;

  // Parse arguments
  for (int i = 1; i < argc; ++i) {
    std::string arg(argv[i]);
    if (arg == "--map") {
      requestMapData = true;
    } else if (arg == "--points") {
      requestMapPoints = true;
    } else if (arg[0] != '-') {
      ipAddress = arg;
    } else {
      std::cerr << "Unknown option: " << arg << std::endl;
    }
  }

  std::cout << "Using S3 IP address : " << ipAddress << std::endl;
  std::cout << "Request map data    : " << (requestMapData   ? "yes" : "no") << std::endl;
  std::cout << "Request map points  : " << (requestMapPoints ? "yes" : "no") << std::endl;



  // Set signal handler
  std::signal(SIGINT,  signalHandler);
  std::signal(SIGTERM, signalHandler);



  // Create S3 node
  S3Node node(ipAddress);



  // Get map data and/or points
  net::MapDataPacketHeader mapDataHeader;
  std::vector<signed char> mapData;
  if (requestMapData) {
    node.requestMapData();
    auto t0 = std::chrono::steady_clock::now();
    while (mapData.empty() && !g_signal.load()) {
      node.getMapData(mapDataHeader, mapData);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(5)) {
        std::cerr << "Timeout waiting map data.\n";
        break;
      }
    }

    std::cout << "Got map data." << std::endl;
    std::cout << "width          = " << mapDataHeader.width << "\n"
              << "height         = " << mapDataHeader.height << "\n"
              << "resolution     = " << mapDataHeader.resolution << "\n"
              << "origin         = [ " << mapDataHeader.originX 
                               << ", " << mapDataHeader.originY
                               << ", " << mapDataHeader.originYaw << " ]\n"
              << "negate         : " << (mapDataHeader.negate ? "yes" : "no") << "\n"
              << "occupiedThresh = " << mapDataHeader.occupiedThresh << "\n"
              << "freeThresh     = " << mapDataHeader.freeThresh << "\n";
    std::cout << "mapDataSize    = " << mapData.size() << "\n";
  }

  std::vector<net::PointXY> mapPoints;
  if (requestMapPoints) {
    node.requestMapPoints();
    auto t0 = std::chrono::steady_clock::now();
    while (mapPoints.empty() && !g_signal.load()) {
      node.getMapPoints(mapPoints);
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (std::chrono::steady_clock::now() - t0 > std::chrono::seconds(5)) {
        std::cerr << "Timeout waiting map points.\n";
        break;
      }
    }

    std::cout << "Got map points." << std::endl;
    std::cout << "mapPointsSize = " << mapPoints.size() << std::endl;
    std::ofstream ofs("map_points.txt");
    if (!ofs) {
      std::cerr << "Failed to open file map_points.txt" << std::endl;
    } else {
      for (const auto& p : mapPoints) {
        ofs << p.x << " " << p.y << "\n";
      }
      ofs.close();
    }
  }



  // Pose reset
  // const float x = 0.0f;   // [m]
  // const float y = 0.0f;   // [m]
  // const float yaw = 0.0f; // [rad]
  // node.resetPose(x, y, z);



  // Send commant to S3
  // Available commands are:
  //   shutdown
  //   disconnect
  //   kill_localizer
  //   restart_odometry
  //   restart_localizer
  //   start_recording_pose
  //   stop_recording_pose
  // node.sendCmd("shutdown");



  // Receive streaming and show
  uint64_t latestOdomStamp           = 0;
  uint64_t latestScanStamp           = 0;
  uint64_t latestImuStamp            = 0;
  uint64_t latestMatchingStatusStamp = 0;

  constexpr float kPi = 3.14159265358979323846f;

  while (!g_signal.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    net::OdomPacket odom;
    if (node.getLatestOdom(latestOdomStamp, odom)) {
      std::cout << "Got new odom." << std::endl;
      std::cout << "x   = " << odom.x << " [m]\n"
                << "y   = " << odom.y << " [m]\n"
                << "yaw = " << odom.yaw * 180.0f / kPi << " [deg]\n"
                << "vx  = " << odom.vx << " [m/s]\n"
                << "vy  = " << odom.vy << " [m/s]\n"
                << "wz  = " << odom.wz * 180.0f / kPi << " [deg/s]\n"; 
    }

    net::ScanPacket scan;
    if (node.getLatestScan(latestScanStamp, scan)) {
      std::cout << "Got new scan." << std::endl;

      // Example to access scan data
      // Note: Scan intensity means confidence of measurement
      // std::ofstream ofs("scan_points.txt");
      // if (!ofs) {
      //   std::cerr << "Failed to open file scan_points.txt" << std::endl;
      // } else {
      //   for (int i = 0; i < 400; ++i) {
      //     const float r = scan.ranges[i];
      //     if (r < scan.rangeMin || scan.rangeMax < r) {
      //       continue;
      //     }
      //     const float t = scan.angleMin + static_cast<float>(i) * scan.angleIncrement;
      //     const float x = r * std::cos(t);
      //     const float y = r * std::sin(t);
      //     ofs << x << " " << " " << y << " " << scan.intensities[i] << "\n";
      //   }
      //   ofs.close();
      // }
    }

    net::IMUPacket imu;
    if (node.getLatestImu(latestImuStamp, imu)) {
      std::cout << "Got new IMU." << std::endl;
      std::cout << "ax = " << imu.ax << " [m/s^2]\n"
                << "ay = " << imu.ay << " [m/s^2]\n"
                << "az = " << imu.az << " [m/s^2]\n"
                << "gx = " << imu.gx * 180.0f / kPi << " [deg/s]\n"
                << "gy = " << imu.gy * 180.0f / kPi << " [deg/s]\n"
                << "gz = " << imu.gz * 180.0f / kPi << " [deg/s]\n"; 
    }

    net::MatchingStatusPacket status;
    if (node.getLatestMatchingStatus(latestMatchingStatusStamp, status)) {
      std::cout << "Got new matching status." << std::endl;
      std::cout << "flag             : " << static_cast<int>(status.flag) << "\n"
                << "num_matches      : " << status.num_matches << "\n"
                << "rmse             : " << status.rmse << " [m]\n"
                << "inlier_ratio     : " << status.inlier_ratio * 100.0f << " [%]\n"
                << "eigenvalue_ratio : " << status.eigenvalue_ratio * 100.0f << " [%]\n";
    }
  }

  node.join();

  return 0;
}
