cmake_minimum_required(VERSION 3.8)
project(rm_nav_assignment)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 纯 C++ 算法核心（不依赖 ROS）
add_library(grid_planner STATIC src/grid_planner.cpp)
target_include_directories(grid_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# ROS2 节点
add_executable(planner_node src/planner_node.cpp)
target_link_libraries(planner_node grid_planner)
ament_target_dependencies(planner_node rclcpp nav_msgs geometry_msgs)

install(TARGETS planner_node grid_planner
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/
  DESTINATION include)
install(DIRECTORY config launch rviz map
  DESTINATION share/${PROJECT_NAME})

ament_package()
