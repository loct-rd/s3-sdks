# S3 SDKs and User Manual

This repository provides Software Development Kits (SDKs) and user manuals for **S3 (Small SLAM Sensor)** developed by **LOCT Co., Ltd.**

The SDKs allow users to communicate with an S3 device, receive SLAM-related data streams (odometry, IMU, LiDAR scans, map data, etc.), and send control commands over a lightweight TCP/IP protocol.

## Supported Development Environments

Currently, this repository supports the following environments:

- **C++ (standalone, without ROS 2)**
  For high-performance and low-level integration using a minimal dependency setup.
- **Python (standalone, without ROS 2)**
  Ideal for rapid prototyping, data analysis, visualization, and scripting.
- **ROS 2 (C++)**
  For seamless integration with the ROS 2 ecosystem, including navigation, visualization, and robotics middleware.

Each SDK provides equivalent core functionality while being tailored to the conventions and strengths of its respective environment.

## License

All SDKs included in this repository are released under the **Apache License 2.0**.