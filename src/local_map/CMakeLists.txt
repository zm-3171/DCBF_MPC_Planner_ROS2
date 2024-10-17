cmake_minimum_required(VERSION 3.8)
project(local_map)

SET(CMAKE_BUILD_TYPE "Release")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

find_package(grid_map_octomap REQUIRED)
find_package(grid_map_ros REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(rviz_visual_tools REQUIRED)

find_package(CGAL REQUIRED)


include_directories(
  include
  ${rclcpp_INCLUDE_DIRS} 
)

set(dependencies
    rclcpp
    std_msgs
    visualization_msgs
    Eigen3
    geometry_msgs
    sensor_msgs
    pcl_conversions
    pcl_ros
    grid_map_octomap
    grid_map_ros
    tf2
    tf2_ros
    tf2_eigen
    rviz_visual_tools
    CGAL
)

# Add executable
add_executable(local_map src/local_map.cpp)
ament_target_dependencies(local_map ${dependencies})

# Link libraries
target_link_libraries(local_map
  ${rclcpp_LIBRARIES}
)

# Install executable
install(TARGETS local_map
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config rviz
  DESTINATION share/${PROJECT_NAME}/)

ament_package()


