cmake_minimum_required(VERSION 3.8)
project(v2x_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(v2x_msg REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(autoware_auto_perception_msgs)
find_package(autoware_auto_geometry_msgs)
find_package(tf2)

# Include headers
include_directories(include)

# Create shared library from vehicle_v2x_node.cpp
add_library(vehicle_v2x_node_lib
  src/vehicle_v2x_node.cpp
)

ament_target_dependencies(vehicle_v2x_node_lib
  rclcpp
  std_msgs
  v2x_msg
  geometry_msgs
  tf2
  autoware_auto_perception_msgs
  autoware_auto_geometry_msgs
)

# vehicle_node executable
add_executable(vehicle_node src/vehicle_node.cpp)
target_link_libraries(vehicle_node vehicle_v2x_node_lib)

ament_target_dependencies(vehicle_node
  rclcpp
  std_msgs
  sensor_msgs
  v2x_msg
  tf2
  geometry_msgs
  autoware_auto_perception_msgs
  autoware_auto_geometry_msgs
)

# Optional: build vehicle_v2x_node executable with a dummy main
# add_executable(vehicle_v2x_node src/v2x_node.cpp)
# target_link_libraries(vehicle_v2x_node vehicle_v2x_node_lib)
# ament_target_dependencies(vehicle_v2x_node rclcpp std_msgs v2x_msg)

# Install executables and library
install(TARGETS
  vehicle_node
  vehicle_v2x_node_lib
  # vehicle_v2x_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
