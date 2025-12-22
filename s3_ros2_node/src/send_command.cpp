/*
 * send_command.cpp
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

#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Usage: ros2 run s3_ros2_node send_command <command>\n";
    rclcpp::shutdown();
    return 1;
  }

  const std::string command = argv[1];

  auto node = std::make_shared<rclcpp::Node>("send_command");

  node->declare_parameter<std::string>("s3_command_topic", "/s3/command");
  std::string commandTopic;
  node->get_parameter("s3_command_topic", commandTopic);
  auto pub = node->create_publisher<std_msgs::msg::String>(commandTopic, 10);

  std_msgs::msg::String msg;
  msg.data = command;

  rclcpp::sleep_for(std::chrono::milliseconds(200));

  pub->publish(msg);

  RCLCPP_INFO(node->get_logger(), "Sent command: '%s'", msg.data.c_str());

  rclcpp::sleep_for(std::chrono::milliseconds(200));

  rclcpp::shutdown();

  return 0;
}