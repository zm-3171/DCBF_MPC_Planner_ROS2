cmake_minimum_required(VERSION 3.8)
project(scene)

SET(CMAKE_BUILD_TYPE "Debug")

ADD_COMPILE_OPTIONS(-std=c++14)
set(CMAKE_CXX_FLAGS "-std=c++14 -O3")

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(gazebo_msgs REQUIRED)

# find_package(rosbag2_transport REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS} 
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${gazebo_msgs_INCLUDE_DIRS}
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
    gazebo_msgs
)

# Add executable
add_executable(pseudo_odom src/pseudo_odom.cpp)
ament_target_dependencies(pseudo_odom ${dependencies})
add_executable(odom_tf src/odom_tf.cpp)
ament_target_dependencies(odom_tf ${dependencies})
add_executable(move_test src/move_test.cpp)
ament_target_dependencies(move_test ${dependencies})

# Link libraries
target_link_libraries(pseudo_odom
  ${rclcpp_LIBRARIES}
)
target_link_libraries(odom_tf
  ${rclcpp_LIBRARIES}
)
target_link_libraries(move_test
  ${rclcpp_LIBRARIES}
)

# Install executable
install(TARGETS pseudo_odom
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS odom_tf
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS move_test
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config worlds
  DESTINATION share/${PROJECT_NAME}/)

ament_package()