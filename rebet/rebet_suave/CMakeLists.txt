cmake_minimum_required(VERSION 3.8)
project(rebet_suave)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies We import external dependencies for both Cpp and Python at the same time.
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rebet_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)



target_link_libraries(rebet_suave_lib
  ${mavros_msgs_LIBRARIES}
)


# Include Cpp "include" directory
include_directories(include)


# Install Cpp executables
install(TARGETS
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include
        FILES_MATCHING PATTERN "*.h")


# Install Python modules
ament_python_install_package(${PROJECT_NAME})


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


ament_export_include_directories(include)

ament_package()
