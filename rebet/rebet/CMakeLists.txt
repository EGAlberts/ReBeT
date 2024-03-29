cmake_minimum_required(VERSION 3.8)
project(rebet)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies We import external dependencies for both Cpp and Python at the same time.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rebet_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(suave_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(wavefront_frontier_msgs REQUIRED)







# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(arborist src/arborist_node.cpp src/system_attribute_value.cpp)

ament_target_dependencies(arborist rclcpp behaviortree_ros2 behaviortree_cpp rebet_msgs lifecycle_msgs diagnostic_msgs geometry_msgs nav_msgs nav2_msgs sensor_msgs slam_toolbox suave_msgs mavros_msgs wavefront_frontier_msgs)

# Install Cpp executables
install(TARGETS
  arborist
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/blackboard_set_client.py
  scripts/blackboard_get_client.py
  scripts/adaptation_manager_node.py
  scripts/system_reflection.py
  scripts/tree_action_client.py
  scripts/save_map_client.py
  scripts/weights_set_client.py
  scripts/yolo_as_a_service.py

  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  worlds
  config
  trees
  DESTINATION share/${PROJECT_NAME}/
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
