cmake_minimum_required(VERSION 3.5)
project(eth2can)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)

# Add your source files here
set(SOURCES
    src/main.cpp
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Create your executable node
add_executable(eth2can_node ${SOURCES})

# Link against ROS 2 libraries
ament_target_dependencies(eth2can_node rclcpp can_msgs)

install(TARGETS
  eth2can_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()