cmake_minimum_required(VERSION 3.8)
project(roslander_gazebo_plugins)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(roslander_wheel_contact SHARED
  src/wheel_contact_plugin.cpp
)

target_include_directories(roslander_wheel_contact PRIVATE
  ${GAZEBO_INCLUDE_DIRS}
)
target_link_libraries(roslander_wheel_contact
  ${GAZEBO_LIBRARIES}
)
ament_target_dependencies(roslander_wheel_contact
  gazebo_ros
  geometry_msgs
)
target_compile_options(roslander_wheel_contact PRIVATE
  ${GAZEBO_CXX_FLAGS}
)

install(TARGETS roslander_wheel_contact
  LIBRARY DESTINATION lib
)

ament_package()
