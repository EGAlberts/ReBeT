cmake_minimum_required(VERSION 3.8)
project(rebet_samples)

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
find_package(rebet REQUIRED)
find_package(std_msgs REQUIRED)
find_package(aal_msgs REQUIRED)


set(THIS_PACKAGE_DEPS
    rclcpp
    rclcpp_lifecycle 
    behaviortree_ros2
    behaviortree_cpp
    rebet_msgs
    rebet
    std_msgs
    aal_msgs)

# Create Cpp executable
add_executable(sample_arborist src/sample_arborist_node.cpp)
ament_target_dependencies(sample_arborist ${THIS_PACKAGE_DEPS})

add_library(snore_plugin SHARED src/snore_pub.cpp)
target_compile_definitions(snore_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(snore_plugin ${THIS_PACKAGE_DEPS})

add_library(wake_plugin SHARED src/wake_pub.cpp)
target_compile_definitions(wake_plugin PRIVATE  BT_PLUGIN_EXPORT )
ament_target_dependencies(wake_plugin ${THIS_PACKAGE_DEPS})

# Install Cpp executables
install(TARGETS
sample_arborist
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/tree_action_client.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  snore_plugin
  wake_plugin
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  snore_plugin
  wake_plugin
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