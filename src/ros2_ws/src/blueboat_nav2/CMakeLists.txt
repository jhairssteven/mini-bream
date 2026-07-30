cmake_minimum_required(VERSION 3.8)
project(blueboat_nav2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  blueboat_nav2/goal_path_planner.py
  blueboat_nav2/scan_stamp_sync.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
