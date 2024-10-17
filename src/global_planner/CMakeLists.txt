cmake_minimum_required(VERSION 3.8)
project(global_planner)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

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
)

# Add executable
add_executable(global_path_publisher src/global_path_publisher.cpp)
ament_target_dependencies(global_path_publisher ${dependencies})

# Link libraries
target_link_libraries(global_path_publisher
  ${rclcpp_LIBRARIES}
)

# Install executable
install(TARGETS global_path_publisher
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/)

ament_package()


