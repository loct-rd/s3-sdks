cmake_minimum_required(VERSION 3.8)
project(s3_ros2_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)



# s3_ros2_node

add_executable(s3_ros2_node
  src/s3_ros2_node.cpp
)

target_include_directories(s3_ros2_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(s3_ros2_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  s3_ros2_node
  "rclcpp"
  "rclpy"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "nav_msgs"
  "tf2_msgs"
  "tf2_ros"
  "tf2_eigen"
  "tf2_geometry_msgs"
)

install(TARGETS s3_ros2_node DESTINATION lib/${PROJECT_NAME})



# send_command

add_executable(send_command
  src/send_command.cpp
)

target_include_directories(send_command PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(send_command PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(
  send_command
  "rclcpp"
  "rclpy"
  "std_msgs"
)

install(TARGETS send_command DESTINATION lib/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
