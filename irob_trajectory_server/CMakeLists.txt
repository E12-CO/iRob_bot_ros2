cmake_minimum_required(VERSION 3.8)
project(irob_trajectory_server)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra -Wpedantic -g")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(iRob_dependencies
	rclcpp
	rclcpp_components
	geometry_msgs
	tf2
	tf2_ros
	tf2_geometry_msgs
	nav_msgs
	std_msgs
)

add_executable(iRob_trajectory_server
	src/iRob_trajectory_server.cpp
)

target_link_libraries(
	iRob_trajectory_server
)

ament_target_dependencies(
	iRob_trajectory_server
	${iRob_dependencies}
)

install(
	TARGETS iRob_trajectory_server
	DESTINATION lib/${PROJECT_NAME}
)

install(
	DIRECTORY params
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
