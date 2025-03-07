cmake_minimum_required(VERSION 3.8)
project(extended_kalman_filter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(msg_2d REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(angles REQUIRED)

include_directories(include)

set(dependencies
  rclcpp
  msg_2d
  geometry_msgs
  tf2
  angles
)

set(visualizer_dependencies
  rclcpp
  visualization_msgs
  msg_2d
)

add_executable(filter_node src/main.cpp)
ament_target_dependencies(filter_node ${dependencies})
target_link_libraries(filter_node Eigen3::Eigen)

add_executable(visualizer src/visualizer.cpp)
ament_target_dependencies(visualizer ${visualizer_dependencies})

install(TARGETS
  filter_node
  visualizer
  DESTINATION lib/${PROJECT_NAME}
)

add_subdirectory(test)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
