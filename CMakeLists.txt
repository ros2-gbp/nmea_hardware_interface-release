cmake_minimum_required(VERSION 3.5)
project(nmea_hardware_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if($ENV{ROS_DISTRO} STREQUAL "galactic")
  add_definitions(-DGALACTIC)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(ros2_control REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(quaternion_operation REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

add_library(nmea_hardware_interface SHARED
  src/gps_hardware_interface.cpp
)

target_include_directories(
  nmea_hardware_interface
  PRIVATE
  include
  ${Boost_INCLUDE_DIRS}
)

target_link_libraries(
  nmea_hardware_interface
  Boost::thread 
)



ament_target_dependencies(nmea_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  ros2_control
  realtime_tools
  geographic_msgs
  quaternion_operation
  nmea_msgs
)
pluginlib_export_plugin_description_file(hardware_interface nmea_hardware_interface.xml)

add_library(nmea_controller_interface SHARED
  src/geopose_publisher.cpp
)

target_include_directories(
  nmea_controller_interface
  PRIVATE
  include
)

ament_target_dependencies(nmea_controller_interface
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  ros2_control
  realtime_tools
  geographic_msgs
)
pluginlib_export_plugin_description_file(controller_interface nmea_controller_interface.xml)

ament_export_libraries(
  nmea_hardware_interface nmea_controller_interface
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  ros2_control
  realtime_tools
  quaternion_operation
  geographic_msgs
  nmea_msgs
)

install(
  TARGETS nmea_hardware_interface nmea_controller_interface
  DESTINATION lib
)


install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
