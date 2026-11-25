cmake_minimum_required(VERSION 3.8)
project(roslander_navigation)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config maps rviz behavior_trees
  DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/eight_point_navigation.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME eight_point_navigation)
ament_package()
