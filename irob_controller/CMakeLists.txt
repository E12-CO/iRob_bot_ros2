cmake_minimum_required(VERSION 3.8)
project(irob_controller)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra -Wpedantic -g")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(irob_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

set(irob_dependencies
	pluginlib
	rclcpp 
	rclcpp_components 
	tf2
	tf2_ros 
	tf2_geometry_msgs 
	tf2_sensor_msgs 
	tf2_eigen 
	geometry_msgs 
	nav_msgs
	irob_msgs
)

# iRob-holonomic plugin
add_library(irob_holonomic_plugins SHARED
	src/iRob_holonomic.cpp
)

ament_target_dependencies(
	irob_holonomic_plugins
	${irob_dependencies}
)

target_include_directories(
	irob_holonomic_plugins PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

ament_export_include_directories(
  include
)

# iRob-Controller node
add_executable(irob_controller src/iRob_controller.cpp)
ament_target_dependencies(
	irob_controller
	${irob_dependencies}
)
target_link_libraries(
	irob_controller
	irob_holonomic_plugins
)
target_include_directories(
	irob_controller PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
	$<INSTALL_INTERFACE:include>
)

install(
	TARGETS irob_controller irob_holonomic_plugins
	DESTINATION lib/${PROJECT_NAME}
)

install(
	DIRECTORY include/
	DESTINATION include
)

install(
	DIRECTORY params
	DESTINATION share/${PROJECT_NAME}
)

ament_package()
