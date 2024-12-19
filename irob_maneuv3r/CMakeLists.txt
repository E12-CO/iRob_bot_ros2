cmake_minimum_required(VERSION 3.8)
project(irob_maneuv3r)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra -Wpedantic -g")

# find dependencies
find_package(ament_cmake REQUIRED)
# ROS2 dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

set(iRob_dependencies
	rclcpp
	rclcpp_components
	geometry_msgs
	tf2
	tf2_ros
	nav_msgs
	sensor_msgs
	irob_msgs
	std_msgs
)

add_executable(iRob_maneuv3r
	src/iRob_interface.cpp
)

target_link_libraries(
	iRob_maneuv3r
)

ament_target_dependencies(
	iRob_maneuv3r
	${iRob_dependencies}
)

install(
	TARGETS iRob_maneuv3r
	DESTINATION lib/${PROJECT_NAME}
)

install(
	DIRECTORY params
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
