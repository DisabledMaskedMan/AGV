cmake_minimum_required(VERSION 3.8)
project(mid360_obstacle_avoidance)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(livox_ros_driver2 REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ObstacleStatus.msg"
  "msg/RoiState.msg"
  "msg/DetectorConfig.msg"
  "srv/SetRoiBoundary.srv"
  "srv/ApplyDetectorConfig.srv"
  DEPENDENCIES std_msgs
)

add_executable(mid360_obstacle_node src/mid360_obstacle_node.cpp)
target_compile_features(mid360_obstacle_node PRIVATE cxx_std_17)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(mid360_obstacle_node PRIVATE -Wall -Wextra -Wpedantic)
endif()
ament_target_dependencies(mid360_obstacle_node rclcpp rcl_interfaces std_msgs std_srvs
  geometry_msgs sensor_msgs visualization_msgs interactive_markers livox_ros_driver2)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(mid360_obstacle_node "${cpp_typesupport_target}")

install(TARGETS mid360_obstacle_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/mid360_web_editor.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch rviz web DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_cluster_detection test/test_cluster_detection.cpp)
  target_include_directories(test_cluster_detection PRIVATE src)
  target_compile_features(test_cluster_detection PRIVATE cxx_std_17)
  add_test(NAME cluster_detection COMMAND test_cluster_detection)
  set_tests_properties(cluster_detection PROPERTIES TIMEOUT 30)
  add_executable(test_roi_editor test/test_roi_editor.cpp)
  target_include_directories(test_roi_editor PRIVATE src)
  target_compile_features(test_roi_editor PRIVATE cxx_std_17)
  add_test(NAME roi_editor COMMAND test_roi_editor)
  set_tests_properties(roi_editor PROPERTIES TIMEOUT 30)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(obstacle_integration test/test_obstacle_node.py TIMEOUT 300
    ENV "MID360_NODE_EXECUTABLE=$<TARGET_FILE:mid360_obstacle_node>")
  ament_add_pytest_test(web_integration test/test_web_editor.py TIMEOUT 180
    ENV "MID360_NODE_EXECUTABLE=$<TARGET_FILE:mid360_obstacle_node>"
        "MID360_WEB_SCRIPT=${CMAKE_CURRENT_SOURCE_DIR}/scripts/mid360_web_editor.py")
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
