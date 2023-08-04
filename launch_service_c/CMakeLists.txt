cmake_minimum_required(VERSION 3.8)
project(launch_service_c)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PYTHON_EXECUTABLE "/usr/bin/python3")
set(PYBIND11_PYTHON_VERSION "3.8")

# find dependencies
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

# this has to go after the pybind stuff
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(launch_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)

add_executable(launch_service_c src/launch_service_c.cpp src/launch_manager_c.cpp src/launch_monitor.cpp src/launch_monitor_child.cpp)
target_include_directories(launch_service_c PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(launch_service_c PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(launch_service_c pybind11::embed)

ament_target_dependencies(launch_service_c
  pybind11
  rclcpp
  rclcpp_action
  launch_msgs
  ament_index_cpp
)

add_executable(bag_service_c src/bag_service_c.cpp src/bag_manager_c.cpp src/bag_node_c.cpp)
target_include_directories(bag_service_c PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(bag_service_c PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(bag_service_c
  rclcpp
  launch_msgs
  rosbag2_cpp
  rosbag2_transport
)

install(TARGETS launch_service_c bag_service_c
  DESTINATION lib/${PROJECT_NAME})

ament_package()
