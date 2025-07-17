cmake_minimum_required(VERSION 3.8)
project(backseat_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Loc.msg"
  "msg/Locg.msg"
  "msg/Telem.msg"
  "action/DoMission.action"
  DEPENDENCIES geographic_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
