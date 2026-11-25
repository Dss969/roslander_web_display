cmake_minimum_required(VERSION 3.8)
project(roslander_bt_nodes)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(roslander_is_goal_nearby_condition_bt_node SHARED
  src/is_goal_nearby_condition.cpp)
target_compile_features(roslander_is_goal_nearby_condition_bt_node PUBLIC cxx_std_17)
target_compile_definitions(roslander_is_goal_nearby_condition_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_include_directories(roslander_is_goal_nearby_condition_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(roslander_is_goal_nearby_condition_bt_node
  behaviortree_cpp_v3 geometry_msgs nav_msgs nav2_util rclcpp tf2_ros)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(roslander_is_goal_nearby_condition_bt_node PRIVATE
    -Wall -Wextra -Wpedantic)
endif()

install(TARGETS roslander_is_goal_nearby_condition_bt_node
  EXPORT export_roslander_bt_nodes
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(tf2_geometry_msgs REQUIRED)
  ament_add_gtest(test_is_goal_nearby test/test_is_goal_nearby.cpp)
  target_link_libraries(test_is_goal_nearby roslander_is_goal_nearby_condition_bt_node)
  ament_target_dependencies(test_is_goal_nearby
    behaviortree_cpp_v3 geometry_msgs nav_msgs rclcpp tf2_ros tf2_geometry_msgs)
endif()

ament_export_targets(export_roslander_bt_nodes HAS_LIBRARY_TARGET)
ament_export_dependencies(behaviortree_cpp_v3 geometry_msgs nav_msgs nav2_util rclcpp tf2_ros)
ament_package()
