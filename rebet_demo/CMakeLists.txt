cmake_minimum_required(VERSION 3.8)
project(rebet_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies We import external dependencies for both Cpp and Python at the same time.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclpy REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(rebet_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(aal_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


set(THIS_PACKAGE_DEPS
    behaviortree_ros2
    std_msgs
    std_srvs
    btcpp_ros2_interfaces 
    geometry_msgs
    nav_msgs
    nav2_msgs
    tf2_geometry_msgs
    rebet_msgs)



# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(arborist src/arborist_node.cpp src/system_attribute_value.cpp)

add_library(patrol_action_plugin SHARED src/patrol_action_node.cpp)
target_compile_definitions(patrol_action_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(patrol_action_plugin ${THIS_PACKAGE_DEPS})

add_library(detect_action_plugin SHARED src/detect_object_service_node.cpp)
target_compile_definitions(detect_action_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(detect_action_plugin ${THIS_PACKAGE_DEPS})

ament_target_dependencies(arborist rclcpp rclcpp_lifecycle behaviortree_ros2 behaviortree_cpp rebet_msgs lifecycle_msgs diagnostic_msgs geometry_msgs nav_msgs nav2_msgs sensor_msgs slam_toolbox aal_msgs)

# Install Cpp executables
install(TARGETS
  arborist
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/system_reflection.py
  scripts/start_tree.py
  scripts/yolo_as_a_service.py
  scripts/change_lighting.py

  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
patrol_action_plugin
detect_action_plugin
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  patrol_action_plugin
  detect_action_plugin
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
  )

install(DIRECTORY
  launch
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
