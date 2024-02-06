cmake_minimum_required(VERSION 3.5)
project(ahrs_can)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)


# Add your source files here
set(SOURCES
    src/main.cpp
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Create your executable node
add_executable(ahrs_can_node ${SOURCES})

# Link against ROS 2 libraries
ament_target_dependencies(ahrs_can_node 
"rclcpp"
"std_msgs"
"sensor_msgs"
"geometry_msgs"
"tf2"
"tf2_ros"
"tf2_msgs"
"can_msgs"
)

install(TARGETS
  ahrs_can_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()