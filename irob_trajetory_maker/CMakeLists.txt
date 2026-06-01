cmake_minimum_required(VERSION 3.8)
project(irob_trajectory_maker)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -Wextra -Wpedantic -g")

find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)

set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
  include/irob_trajectory_maker/irob_trajectory_maker_panel.hpp
)

add_library(trajectory_panel src/irob_trajectory_maker_panel.cpp ${MOC_FILES})
target_include_directories(trajectory_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(trajectory_panel
  pluginlib
  rviz_common
)
install(TARGETS trajectory_panel
        EXPORT export_irob_trajectory_maker
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include
)
install(FILES rviz_common_plugins.xml
        DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/icons"
  DESTINATION "share/${PROJECT_NAME}"
)

ament_export_include_directories(include)
ament_export_targets(export_irob_trajectory_maker)
pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)

ament_package()