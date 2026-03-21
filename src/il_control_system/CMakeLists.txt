cmake_minimum_required(VERSION 3.10)
project(il_control_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Add executables
add_executable(il_control_system_refactored
  src/il_control_system_refactored.cpp
)

add_executable(offboard_manager_node
  src/offboard_manager_node.cpp
)

add_executable(trajectory_planner_node
  src/trajectory_planner_node.cpp
)

# Link dependencies
ament_target_dependencies(il_control_system_refactored
  rclcpp
  geometry_msgs
  px4_msgs
  std_msgs

)

ament_target_dependencies(trajectory_planner_node
  rclcpp
  geometry_msgs
  px4_msgs
)
target_link_libraries(trajectory_planner_node yaml-cpp)

ament_target_dependencies(offboard_manager_node
  rclcpp
  geometry_msgs
  px4_msgs
)

# Install executables
install(TARGETS 
  il_control_system_refactored
  offboard_manager_node
  trajectory_planner_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include/
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch/
)

# ============================================================================
# FIX: Install config directory (THIS WAS MISSING!)
# ============================================================================
# This copies config/il_control_parameters.yaml to the install directory
# so the launch file can find it
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config/
)

ament_package()