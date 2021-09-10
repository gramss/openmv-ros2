cmake_minimum_required(VERSION 3.5)
project(openmv_bt_plugins CXX)

find_package(ament_cmake REQUIRED)
find_package(nav2_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(openmv_msgs REQUIRED)
find_package(nav2_behavior_tree)

# nav2_package()

include_directories(
  include
)


set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  sensor_msgs
  behaviortree_cpp_v3
  std_msgs
  std_srvs
  nav2_util
  openmv_msgs
  nav2_behavior_tree
)

add_library(openmv_setup_cam_normal_bt_node SHARED plugins/action/setup_cam_normal.cpp)
list(APPEND plugin_libs openmv_setup_cam_normal_bt_node)

# add_library(openmv_setup_cam_FLIR_bt_node SHARED plugins/action/compute_path_to_pose_action.cpp)
# list(APPEND plugin_libs openmv_setup_cam_FLIR_bt_node)

# add_library(openmv_reset_cam_serial_bt_node SHARED plugins/action/compute_path_to_pose_action.cpp)
# list(APPEND plugin_libs openmv_reset_cam_serial_bt_node)

# add_library(openmv_reset_start_movement_stream_bt_node SHARED plugins/action/compute_path_to_pose_action.cpp)
# list(APPEND plugin_libs openmv_reset_start_movement_stream_bt_node)

# add_library(openmv_reset_start_normal_stream_bt_node SHARED plugins/action/compute_path_to_pose_action.cpp)
# list(APPEND plugin_libs openmv_reset_start_normal_stream_bt_node)

# add_library(openmv_reset_sanity_check_bt_node SHARED plugins/action/compute_path_to_pose_action.cpp)
# list(APPEND plugin_libs openmv_reset_sanity_check_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES openmw_tree_nodes.xml DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  #add_subdirectory(test)
endif()

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${plugin_libs}
)

ament_export_dependencies(${dependencies})

ament_package()
