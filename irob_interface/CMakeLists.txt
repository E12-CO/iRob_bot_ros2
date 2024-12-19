cmake_minimum_required(VERSION 3.8)
project(irob_interface)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra -Wpedantic -g")

# find dependencies
find_package(ament_cmake REQUIRED)
# ROS2 dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Custom message for motor
find_package(irob_msgs REQUIRED)

set(iRob_dependencies
	rclcpp
	rclcpp_components
	geometry_msgs
	nav_msgs
	sensor_msgs
	irob_msgs
	std_msgs
)

add_executable(iRob_interface
	src/iRob_interface.cpp
)

target_link_libraries(
	iRob_interface
)

ament_target_dependencies(
	iRob_interface
	${iRob_dependencies}
)

install(
	TARGETS iRob_interface
	DESTINATION lib/${PROJECT_NAME}
)

install(
	DIRECTORY params
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
