cmake_minimum_required(VERSION 3.8)
project(btick)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(pugixml REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(
  include
)

add_library(${PROJECT_NAME}_core SHARED
  src/tree_parser.cpp
  src/tree_instance.cpp
  src/logs.cpp
  src/monitor.cpp
)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_core
  pugixml::pugixml
  nlohmann_json::nlohmann_json
)
ament_target_dependencies(${PROJECT_NAME}_core
  rclcpp
  std_msgs
  behaviortree_cpp
)

# Install library
install(TARGETS ${PROJECT_NAME}_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(${PROJECT_NAME}_core)
ament_export_include_directories(include)
ament_export_dependencies(nlohmann_json)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_blackboard_write test/test_blackboard_write.cpp)
  target_link_libraries(test_blackboard_write ${PROJECT_NAME}_core)
  ament_target_dependencies(test_blackboard_write
    rclcpp
    behaviortree_cpp
  )

  ament_add_gtest(test_node_port_edit test/test_node_port_edit.cpp)
  target_link_libraries(test_node_port_edit ${PROJECT_NAME}_core)
  ament_target_dependencies(test_node_port_edit
    rclcpp
    behaviortree_cpp
  )

  ament_add_gtest(test_output_ports_isolation test/test_output_ports_isolation.cpp)
  target_link_libraries(test_output_ports_isolation ${PROJECT_NAME}_core)
  ament_target_dependencies(test_output_ports_isolation
    rclcpp
    behaviortree_cpp
  )

  ament_add_gtest(test_event_timestamp_precision test/test_event_timestamp_precision.cpp)
  target_link_libraries(test_event_timestamp_precision ${PROJECT_NAME}_core)
  ament_target_dependencies(test_event_timestamp_precision
    rclcpp
    behaviortree_cpp
  )

  ament_add_gtest(test_json_utils test/test_json_utils.cpp)

  ament_add_gtest(test_fault_injection test/test_fault_injection.cpp)
  target_link_libraries(test_fault_injection ${PROJECT_NAME}_core)
  ament_target_dependencies(test_fault_injection
    rclcpp
    behaviortree_cpp
  )

  ament_add_gtest(test_tree_parser test/test_tree_parser.cpp)
  target_link_libraries(test_tree_parser ${PROJECT_NAME}_core)
  ament_target_dependencies(test_tree_parser
    rclcpp
    behaviortree_cpp
  )

  ament_add_gtest(test_groot2_btick_isolation test/test_groot2_btick_isolation.cpp)
  target_link_libraries(test_groot2_btick_isolation ${PROJECT_NAME}_core zmq)
  ament_target_dependencies(test_groot2_btick_isolation
    rclcpp
    behaviortree_cpp
  )

  ament_add_gtest(test_logs test/test_logs.cpp)
  target_link_libraries(test_logs ${PROJECT_NAME}_core)
  ament_target_dependencies(test_logs
    rclcpp
    behaviortree_cpp
  )

  ament_add_gtest(test_multi_monitor_routing test/test_multi_monitor_routing.cpp)
  target_link_libraries(test_multi_monitor_routing ${PROJECT_NAME}_core)
  ament_target_dependencies(test_multi_monitor_routing
    rclcpp
    std_msgs
    behaviortree_cpp
  )

  add_executable(test_monitor_helper test/test_monitor_helper.cpp)
  target_link_libraries(test_monitor_helper ${PROJECT_NAME}_core)
  ament_target_dependencies(test_monitor_helper
    rclcpp
    behaviortree_cpp
  )
endif()

ament_package()
