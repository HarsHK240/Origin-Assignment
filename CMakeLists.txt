cmake_minimum_required(VERSION 3.5)
project(assignment)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(trajectory_node 
  src/main.cpp
  src/robot_navigator_node.cpp
  src/path_planner.cpp
  src/trajectory_tracker.cpp)
ament_target_dependencies(trajectory_node rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs)

include_directories(include)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install executable
install(TARGETS
  trajectory_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  
  # Create the test executable
  ament_add_gtest(test_path_planner test/test_path_planner.cpp)
  
  # Tell the test where to find your headers
  target_include_directories(test_path_planner PUBLIC include)
  
  # Link the test to the actual path planner logic so it can run the functions
  target_sources(test_path_planner PRIVATE src/path_planner.cpp)
endif()


ament_package()
