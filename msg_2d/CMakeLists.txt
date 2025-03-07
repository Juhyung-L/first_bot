cmake_minimum_required(VERSION 3.8)
project(msg_2d)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(msg_2d
  "msg/Pose.msg"
  "msg/Velocity.msg"
  "msg/OdometryStamped.msg"
  "msg/ImuStamped.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
