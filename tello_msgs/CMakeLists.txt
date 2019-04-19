cmake_minimum_required(VERSION 3.5)
project(tello_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

## Emulate colcon in CLion
#if($ENV{CLION_IDE})
#  message("Running inside CLion")
#  find_package(fastrtps_cmake_module REQUIRED)
#  set(FastRTPS_INCLUDE_DIR "/opt/ros/crystal/include")
#  set(FastRTPS_LIBRARY_RELEASE "/opt/ros/crystal/lib/libfastrtps.so")
#endif()
#
## Debugging: set _dump_all_variables to true
#set(_dump_all_variables false)
#if(_dump_all_variables)
#  get_cmake_property(_variable_names VARIABLES)
#  list(SORT _variable_names)
#  foreach(_variable_name ${_variable_names})
#    message(STATUS "${_variable_name}=${${_variable_name}}")
#  endforeach()
#endif()
#
## Find packages
#find_package(ament_cmake REQUIRED)
#find_package(rosidl_default_generators REQUIRED)
#find_package(std_msgs REQUIRED)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(DIRECTORY msg FILES FlightData.msg TelloTelemetry.msg TelloResponse.msg TelloAction.msg)
add_service_files(DIRECTORY srv FILES TelloCommand.srv)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(${catkin_INCLUDE_DIRS})
# # Generate ROS interfaces
# rosidl_generate_interfaces(
#   ${PROJECT_NAME}
#   "msg/FlightData.msg"
#   "msg/TelloResponse.msg"
#   "srv/TelloAction.srv"
#   DEPENDENCIES std_msgs
# )
# 
# ament_package()
