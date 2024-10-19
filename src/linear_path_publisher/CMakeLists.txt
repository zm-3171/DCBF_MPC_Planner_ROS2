cmake_minimum_required(VERSION 3.8)
project(linear_path_publisher)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# find_package(rosbag2_transport REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS} 
)

set(dependencies
    rclcpp
    std_msgs
    nav_msgs
    geometry_msgs
    Eigen3
    tf2
    tf2_ros
    tf2_geometry_msgs
)

# Add executable
add_executable(linear_path_publisher src/linear_path_publisher.cpp)
ament_target_dependencies(linear_path_publisher ${dependencies})

# Link libraries
target_link_libraries(linear_path_publisher
  ${rclcpp_LIBRARIES}
)

# Install executable
install(TARGETS linear_path_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/)

ament_package()


