cmake_minimum_required(VERSION 3.8)
project(hello_gazebo_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

# Install parameter files
install(DIRECTORY params DESTINATION share/${PROJECT_NAME}/)

# Install RViz config files
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME}/)

# Install map files
install(DIRECTORY map DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
