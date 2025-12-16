cmake_minimum_required(VERSION 3.8)
project(datasync_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Default to C++17 for ROS 2 Humble
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Event messages (optional at configure-time; but required to build this node)
find_package(dvs_msgs REQUIRED)

add_executable(motion_compensation_node src/motion_compensation_node.cpp)
ament_target_dependencies(motion_compensation_node
  rclcpp
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
  dvs_msgs
)

target_include_directories(motion_compensation_node PRIVATE
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(motion_compensation_node
  ${OpenCV_LIBRARIES}
)

install(TARGETS motion_compensation_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
