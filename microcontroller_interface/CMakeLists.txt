cmake_minimum_required(VERSION 3.8)
project(microcontroller_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(msg_2d REQUIRED)
find_package(tf2 REQUIRED)
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

set(dependencies
  rclcpp
  geometry_msgs
  msg_2d
  tf2
)

include_directories(include)

add_executable(microcontroller_interface src/main.cpp)
ament_target_dependencies(microcontroller_interface ${dependencies})
target_link_libraries(microcontroller_interface ${SERIAL_LDFLAGS})
target_link_directories(microcontroller_interface PRIVATE ${SERIAL_INCLUDE_DIRS})

install(TARGETS
  microcontroller_interface
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
