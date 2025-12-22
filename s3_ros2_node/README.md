### How to Use

Please copy the **s3_ros2_node** package into your ROS 2 workspace.

```shell
cd your/ros2/ws
colcon build
source install/setup.bash
ros2 launch s3_ros2_node s3_ros2_node.launch.py
```

By default, the ROS 2 node connects to **192.168.11.100**.

Parameters such as the S3 IP address can be found in **config/s3_config.yaml**. Please edit this file to match your environment.

------

### Subscribed Topics

- **geometry_msgs::msg::PoseWithCovarianceStamped** : `/initialpose` (fixed)

  Reset the S3 pose using RViz.

- **std_msgs::msg::String** : `/s3/command` (default)

  Send commands to the S3 device.

Commands can also be sent to S3 using the utility node included in this package:

```shell
ros2 run s3_ros2_node send_command <command>
```

Replace `<command>` with one of the following:

- `shutdown`
- `disconnect`
- `kill_localizer`
- `restart_odometry`
- `restart_localizer`
- `start_recording_pose`
- `stop_recording_pose`

------

### Published Topics

- **nav_msgs::msg::OccupancyGrid** : `/s3/map` (default)

  Occupancy grid map.

- **sensor_msgs::msg::PointCloud** : `/s3/map_points` (default)

  Map points.

- **sensor_msgs::msg::LaserScan** : `/s3/scan` (default)

  LiDAR scan data.

- **sensor_msgs::msg::PointCloud** : `/s3/point_cloud` (default)

  LiDAR point cloud data.

- **sensor_msgs::msg::Imu** : `/s3/imu` (default)

  Raw IMU data.

- **nav_msgs::msg::Odometry** : `/s3/odom` (default)

  Odometry data including position, orientation, linear velocity, and angular velocity.

- **std_msgs::msg::Int32** : `/s3/ms/flag` (default)

  Matching status flag.

- **std_msgs::msg::Int32** : `/s3/ms/num_matches` (default)

  Number of scan points matched with the map.

- **std_msgs::msg::Float64** : `/s3/ms/rmse` (default)

  Root Mean Square Error (RMSE) of scan-to-map matching.

- **std_msgs::msg::Float64** : `/s3/ms/inlier_ratio` (default)

  Inlier ratio of the current scan.

- **std_msgs::msg::Float64** : `/s3/ms/eigenvalue_ratio` (default)

  Eigenvalue ratio of the Hessian matrix related to translation.