cmake_minimum_required(VERSION 3.8)
project(launch_service_c)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(pybind11 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(launch_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rclcpp REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(launch_service_c src/launch_service_c.cpp src/launch_manager_c.cpp)
target_include_directories(launch_service_c PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(launch_service_c PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_link_libraries(pybind11::embed)

ament_target_dependencies(launch_service_c  
  rclcpp
  pybind11
  launch_msgs
  geometry_msgs
  ament_index_cpp
  rclcpp_action
  sensor_msgs
  std_msgs
  std_srvs
)

install(TARGETS launch_service_c
  DESTINATION lib/${PROJECT_NAME})

ament_package()
