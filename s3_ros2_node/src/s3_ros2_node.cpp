/*
 * s3_ros2_node.cpp
 *
 * S3 ROS 2 Node
 *
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
 * Notes:
 * - This package provides a ROS 2 interface (topics/TF) for the S3 device.
 * - S3 is a product of LOCT Co., Ltd.
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

#include <unistd.h>
#include <signal.h>
#include <arpa/inet.h>
#include <netinet/tcp.h>

#include <mutex>
#include <thread>
#include <atomic>
#include <cstring>
#include <iostream>
#include <vector>
#include <chrono>
#include <cstdint>

#include <s3_ros2_node/net_utils.h>

class S3Ros2Node : public rclcpp::Node {
 public:
  S3Ros2Node():
    Node("s3_ros2_node")
  {
    // Subscriber
    initialposeSub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", rclcpp::SensorDataQoS(),
      std::bind(&S3Ros2Node::initialPoseCallback, this, std::placeholders::_1));
    
    this->declare_parameter<std::string>("s3_command_topic", "/s3/command");
    std::string commandTopic;
    this->get_parameter("s3_command_topic", commandTopic);
    commandSub_ = this->create_subscription<std_msgs::msg::String>(
      commandTopic, rclcpp::SensorDataQoS(),
      std::bind(&S3Ros2Node::commandCallback, this, std::placeholders::_1));

    // Publusher
    mapDataPublished_ = false;
    this->declare_parameter<std::string>("s3_map_data_topic", "/s3/map");
    std::string mapDataTopic;
    this->get_parameter("s3_map_data_topic", mapDataTopic);
    rclcpp::QoS mapQos(rclcpp::KeepLast(1));
    mapQos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    mapQos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);
    mapDataPub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(mapDataTopic, mapQos);

    mapPointsPublished_ = false;
    this->declare_parameter<std::string>("s3_map_points_topic", "/s3/map_points");
    std::string mapPointsTopic;
    this->get_parameter("s3_map_points_topic", mapPointsTopic);
    mapPointsPub_ = this->create_publisher<sensor_msgs::msg::PointCloud>(mapPointsTopic, mapQos);

    this->declare_parameter<std::string>("s3_scan_topic", "/s3/scan");
    std::string scanTopic;
    this->get_parameter("s3_scan_topic", scanTopic);
    scanPub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scanTopic, 100);

    this->declare_parameter<std::string>("s3_point_cloud_topic", "/s3/point_cloud");
    std::string pointCloudTopic;
    this->get_parameter("s3_point_cloud_topic", pointCloudTopic);
    pointCloudPub_ = this->create_publisher<sensor_msgs::msg::PointCloud>(pointCloudTopic, 100);

    this->declare_parameter<std::string>("s3_imu_topic", "/s3/imu");
    std::string imuTopic;
    this->get_parameter("s3_imu_topic", imuTopic);
    imuPub_ = this->create_publisher<sensor_msgs::msg::Imu>(imuTopic, 100);

    this->declare_parameter<std::string>("s3_odom_topic", "/s3/odom");
    std::string odomTopic;
    this->get_parameter("s3_odom_topic", odomTopic);
    odomPub_ = this->create_publisher<nav_msgs::msg::Odometry>(odomTopic, 100);

    // Load S3 parameters
    // this->declare_parameter<std::string>("s3_ip_address", "127.0.0.1");
    this->declare_parameter<std::string>("s3_ip_address", "192.168.11.100");
    this->get_parameter("s3_ip_address", s3IpAddress_);

    // TF
    this->declare_parameter<bool>("broadcast_tf", true);
    this->get_parameter("broadcast_tf", broadcastTf_);
    if (broadcastTf_) {
      tfBroadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      this->declare_parameter<std::string>("s3_source_frame", "s3_odom");
      this->declare_parameter<std::string>("s3_child_frame", "s3_lidar");
      this->get_parameter("s3_source_frame", s3SourceFrame_);
      this->get_parameter("s3_child_frame", s3ChildFrame_);
    }

    // Initialization for socket communication
    setupSocket();
    clientRunning_.store(true);
    clientThread_ = std::thread(&S3Ros2Node::clientLoop, this);

    // Show parameters
    RCLCPP_INFO(this->get_logger(), "s3_command_topic:     %s", commandTopic.c_str());

    RCLCPP_INFO(this->get_logger(), "s3_map_data_topic:    %s", mapDataTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "s3_map_points_topic:  %s", mapPointsTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "s3_scan_topic:        %s", scanTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "s3_point_cloud_topic: %s", pointCloudTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "s3_imu_topic:         %s", imuTopic.c_str());
    RCLCPP_INFO(this->get_logger(), "s3_odom_topic:        %s", odomTopic.c_str());

    RCLCPP_INFO(this->get_logger(), "s3_ip_address:        %s", s3IpAddress_.c_str());

    RCLCPP_INFO(this->get_logger(), "broadcast_tf:         %s", broadcastTf_ ? "yes" : "no");
    RCLCPP_INFO(this->get_logger(), "s3_source_frame:      %s", s3SourceFrame_.c_str());
    RCLCPP_INFO(this->get_logger(), "s3_child_frame:       %s", s3ChildFrame_.c_str());
  }

  ~S3Ros2Node() override {
    clientRunning_.store(false);
    ::shutdown(sock_, SHUT_RDWR);
    ::close(sock_);
    if (clientThread_.joinable()) {
      clientThread_.join();
    }
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

  void commandCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(mutex_);
    s3Command_ = *msg;
    s3CommandUpdated_.store(true);
  }

  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
    resetPoseX_ = static_cast<float>(msg->pose.pose.position.x);
    resetPoseY_ = static_cast<float>(msg->pose.pose.position.y);
    tf2::Quaternion q(
      msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y,
      msg->pose.pose.orientation.z,
      msg->pose.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    resetPoseYaw_ = static_cast<float>(yaw);
    resetPose_.store(true);
    RCLCPP_INFO(this->get_logger(), "Get pose reset signal.");
  }

  void clientLoop() {
    std::atomic<bool> run(true);

    while (rclcpp::ok()) {
      std::thread rx(&S3Ros2Node::rxLoop, this, sock_, std::ref(run));
      std::thread tx(&S3Ros2Node::txLoop, this, sock_, std::ref(run));
      rx.join();
      run.store(false);
      tx.join();

      if (rclcpp::ok()) {
        cleanupSocket();
        setupSocket();
        run.store(true);
        RCLCPP_INFO(this->get_logger(), "Restart client loop.");
      }
    }

    rclcpp::shutdown();
  }

  void rxLoop(
    int sock,
    std::atomic<bool>& run)
  {
    while (run.load() && rclcpp::ok()) {
      net::Header h{};
      std::vector<uint8_t> pl;
      if (!net::recvFrame(sock, h, pl)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to receive data from S3.");
        break;
      }

      // const uint32_t seq = ntohl(h.seq);
      const uint64_t stamp = net::ntohll(h.stamp);
      const net::MsgType type = net::toMsgType(h.type);
      const net::StreamId streamId = net::toStreamId(h.stream_id);
      if (streamId == net::StreamId::TELEMETRY) {
        latestStreamingStamp_ = stamp;
        // std::cout << "Latest stareaming stamp: " << latestStreamingStamp_ << std::endl;
      }

      if (type == net::MsgType::ERROR) {
        RCLCPP_ERROR(this->get_logger(), "Got error message from S3.");
        break;

      } else if (type == net::MsgType::ACK) {
        RCLCPP_INFO(this->get_logger(), "Got ACK.");

      } else if (type == net::MsgType::MAP_DATA) {
        if (pl.size() == 0) {
          RCLCPP_WARN(this->get_logger(), "Map data payload is empty.");
          continue;
        }
        net::MapDataPacketHeader header;
        std::vector<signed char> data;
        if (!parseMapDataPayload(pl, header, data)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to parse map data payload.");
          break;
        }
        publishMapData(mapDataPub_, s3SourceFrame_, header, data);
        mapDataPublished_ = true;

      } else if (type == net::MsgType::MAP_POINTS) {
        if (pl.size() == 0) {
          RCLCPP_WARN(this->get_logger(), "Map points payload is empty.");
          continue;
        }
        net::PointsPacketHeader header;
        std::vector<net::PointXY> points;
        if (!parseMapPointsPayload(pl, header, points)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to parse map points payload.");
          break;
        }
        publishMapPoints(mapPointsPub_, s3SourceFrame_, points);
        mapPointsPublished_ = true;

      } else if (type == net::MsgType::ODOMETRY) {
        net::OdomPacket pkt;
        if (!parseOdomPayload(pl, pkt)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to parse odometry payload.");
          break;
        }
        publishOdom(odomPub_, s3SourceFrame_, s3ChildFrame_, pkt);
        if (broadcastTf_) {
          publishOdomTF(s3SourceFrame_, s3ChildFrame_, pkt);
        }

      } else if (type == net::MsgType::SCAN) {
        net::ScanPacket pkt;
        if (!parseScanPayload(pl, pkt)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to scan payload.");
          break;
        }
        publishScan(scanPub_, s3ChildFrame_, pkt);
        publishPointCloud(pointCloudPub_, s3ChildFrame_, pkt);

      } else if (type == net::MsgType::IMU) {
        net::IMUPacket pkt;
        if (!parseIMUPayload(pl, pkt)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to parse IMU payload.");
          break;
        }
        publishImu(imuPub_, s3ChildFrame_, pkt);

      } else if (type == net::MsgType::MATCHING_STATUS) {
        net::MatchingStatusPacket pkt;
        if (!parseMatchingStatusPayload(pl, pkt)) {
          RCLCPP_ERROR(this->get_logger(), "Failed to parse matchig status payload.");
          break;
        }
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

  void publishMapData(
    const rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub,
    const std::string& frame_id,
    const net::MapDataPacketHeader& header,
    const std::vector<signed char>& data)
  {
    nav_msgs::msg::OccupancyGrid grid;
    grid.header.stamp = rclcpp::Time();//rclcpp::Time(stamp);
    grid.header.frame_id = frame_id;
    grid.info.map_load_time = rclcpp::Time();//rclcpp::Time(stamp);
    grid.info.resolution = header.resolution;
    grid.info.width = static_cast<uint32_t>(header.width);
    grid.info.height = static_cast<uint32_t>(header.height);
    grid.info.origin.position.x = header.originX;
    grid.info.origin.position.y = header.originY;
    grid.info.origin.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, header.originYaw);
    q.normalize();
    grid.info.origin.orientation.x = q.x();
    grid.info.origin.orientation.y = q.y();
    grid.info.origin.orientation.z = q.z();
    grid.info.origin.orientation.w = q.w();

    grid.data.resize(data.size());
    const int w = header.width;
    const int h = header.height;
    for (int y = 0; y < h; ++y) {
      for (int x = 0; x < w; ++x) {
        const int src_y = h - 1 - y;
        const int src_idx = src_y * w + x;
        const int dst_idx = y * w + x;
        grid.data[dst_idx] = static_cast<int8_t>(data[src_idx]);
      }
    }
    pub->publish(grid);
  }

  void publishMapPoints(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub,
    const std::string& frame_id,
    const std::vector<net::PointXY>& points)
  {
    sensor_msgs::msg::PointCloud cloud;
    cloud.header.stamp = rclcpp::Time();//rclcpp::Time(stamp);
    cloud.header.frame_id = frame_id;
    for (const auto& pt: points) {
      geometry_msgs::msg::Point32 p;
      p.x = pt.x;
      p.y = pt.y;
      cloud.points.emplace_back(p);
    }
    pub->publish(cloud);
  }

  void publishOdom(
    const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub,
    const std::string& frame_id,
    const std::string& child_frame_id, net::OdomPacket pkt)
  {
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = rclcpp::Time();//rclcpp::Time(stamp);
    odom.header.frame_id = frame_id;
    odom.child_frame_id  = child_frame_id;
    odom.pose.pose.position.x = pkt.x;
    odom.pose.pose.position.y = pkt.y;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pkt.yaw);
    q.normalize();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.orientation.w = q.w();
    odom.twist.twist.linear.x = pkt.vx;
    odom.twist.twist.linear.y = pkt.vy;
    odom.twist.twist.angular.z = pkt.wz;
    pub->publish(odom);
  }

  void publishOdomTF(
    const std::string& frame_id,
    const std::string& child_frame_id,
    const net::OdomPacket& pkt)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = rclcpp::Time();//rclcpp::Time(stamp);
    tf_msg.header.frame_id = frame_id;
    tf_msg.child_frame_id = child_frame_id;
    tf_msg.transform.translation.x = pkt.x;
    tf_msg.transform.translation.y = pkt.y;
    tf_msg.transform.translation.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, pkt.yaw);
    q.normalize();
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    tfBroadcaster_->sendTransform(tf_msg);
  }

  void publishScan(
    const rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub,
    const std::string& frame_id,
    net::ScanPacket pkt)
  {
    sensor_msgs::msg::LaserScan scan;
    scan.header.stamp = rclcpp::Time();//rclcpp::Time(stamp);
    scan.header.frame_id = frame_id;
    scan.angle_min = pkt.angleMin;
    scan.angle_max = pkt.angleMax;
    scan.angle_increment = pkt.angleIncrement;
    scan.time_increment = pkt.timeIncrement;
    scan.scan_time = pkt.scanTime;
    scan.range_min = pkt.rangeMin;
    scan.range_max = pkt.rangeMax;
    const size_t size = 400;
    scan.ranges.resize(size);
    scan.intensities.resize(size);
    for (size_t i = 0; i < size; ++i) {
      scan.ranges[i] = pkt.ranges[i];
      scan.intensities[i] = pkt.intensities[i];
    }
    pub->publish(scan);
  }

  void publishPointCloud(
    const rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub,
    const std::string& frame_id,
    net::ScanPacket pkt)
  {
    sensor_msgs::msg::PointCloud cloud;
    cloud.header.stamp = rclcpp::Time();//rclcpp::Time(stamp);
    cloud.header.frame_id = frame_id;
    cloud.points.reserve(400);
    for (int i = 0; i < 400; ++i) {
      const float r = pkt.ranges[i];
      if (r < pkt.rangeMin || pkt.rangeMax < r) {
        continue;
      }

      const float a = pkt.angleMin + static_cast<float>(i) * pkt.angleIncrement;
      geometry_msgs::msg::Point32 p;
      p.x = r * std::cos(a);
      p.y = r * std::sin(a);
      cloud.points.emplace_back(p);
    }
    pub->publish(cloud);
  }
  
  void publishImu(
      const rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub,
      const std::string& frame_id,
      const net::IMUPacket& pkt)
  {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = rclcpp::Time();//rclcpp::Time(stamp);
    imu.header.frame_id = frame_id;
    imu.angular_velocity.x = pkt.gx;
    imu.angular_velocity.y = pkt.gy;
    imu.angular_velocity.z = pkt.gz;
    imu.linear_acceleration.x = pkt.ax;
    imu.linear_acceleration.y = pkt.ay;
    imu.linear_acceleration.z = pkt.az;
    pub->publish(imu);
  }

  void txLoop(
    int sock,
    std::atomic<bool>& run)
  {
    using namespace std::chrono_literals;
    uint32_t seq = 0;

    net::sendFrame(sock, net::MsgType::CMD, net::StreamId::CONTROL, seq, "start_streaming");

    while (run.load() && rclcpp::ok()) {
      std::this_thread::sleep_for(1s);

      if (!mapDataPublished_) {
        net::sendFrame(sock, net::MsgType::CMD, net::StreamId::CONTROL, seq, "get_map_data");
      }

      if (!mapPointsPublished_) {
        net::sendFrame(sock, net::MsgType::CMD, net::StreamId::CONTROL, seq, "get_map_points");
      }

      if (s3CommandUpdated_.load()) {
        net::sendFrame(sock, net::MsgType::CMD, net::StreamId::CONTROL, seq, s3Command_.data);
        s3CommandUpdated_.store(false);
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
        const uint64_t stamp = rclcpp::Time().nanoseconds();
        if (!net::sendFrame(sock, stamp, net::MsgType::POSE,
          net::StreamId::TELEMETRY, seq, payload.data(), payloadSize))
        {
          std::cerr << "Failed to send reset pose." << std::endl;
          run = false;
          break;
        }
        resetPose_.store(false);
        RCLCPP_INFO(this->get_logger(), "Sent reset pose command.");
      }
    }

    net::sendFrame(sock, net::MsgType::CMD, net::StreamId::CONTROL, seq, "stop_streaming");
    sleep(1);
    run.store(false);
  }

  int sock_;
  std::thread clientThread_;
  std::atomic<bool> clientRunning_{false};
  uint64_t latestStreamingStamp_;

  std::string s3IpAddress_;

  mutable std::mutex mutex_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr commandSub_;
  std_msgs::msg::String s3Command_;
  std::atomic<bool> s3CommandUpdated_{false};

  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialposeSub_;
  std::atomic<bool> resetPose_{false};
  float resetPoseX_;
  float resetPoseY_;
  float resetPoseYaw_;

  bool mapDataPublished_{false};
  bool mapPointsPublished_{false};
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mapDataPub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr mapPointsPub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scanPub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pointCloudPub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imuPub_;

  bool broadcastTf_{true};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster_;
  std::string s3SourceFrame_;
  std::string s3ChildFrame_;
}; // class S3Ros2Node

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<S3Ros2Node>());
  rclcpp::shutdown();
  return 0;
}
