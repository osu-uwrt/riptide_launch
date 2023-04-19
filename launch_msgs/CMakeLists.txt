cmake_minimum_required(VERSION 3.8)
project(launch_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Grab all message files 
file(GLOB msg_sources RELATIVE ${PROJECT_SOURCE_DIR} msg/*.msg)
file(GLOB srv_sources RELATIVE ${PROJECT_SOURCE_DIR} srv/*.srv)
file(GLOB action_sources RELATIVE ${PROJECT_SOURCE_DIR} action/*.action)
# message("Grabbing messages ${action_sources}")

#generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_sources}
  ${srv_sources}
  ${action_sources}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
