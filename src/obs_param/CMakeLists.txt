cmake_minimum_required(VERSION 3.8)
project(obs_param)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# find_package(rosbag2_transport REQUIRED)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS} 
)

set(dependencies
    rclcpp
    std_msgs
    visualization_msgs
    Eigen3
    # rosbag2_transport 
)

# Add executable
add_executable(obs_kf src/obs_kf.cpp src/kalman.cpp)
ament_target_dependencies(obs_kf ${dependencies})

# Link libraries
target_link_libraries(obs_kf
  ${rclcpp_LIBRARIES}
)

# Install executable
install(TARGETS obs_kf
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


