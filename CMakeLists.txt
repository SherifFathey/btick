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
target_include_directories(${PROJECT_NAME}_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
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
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(${PROJECT_NAME}_core)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  rclcpp
  std_msgs
  behaviortree_cpp
  pugixml
  nlohmann_json
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
