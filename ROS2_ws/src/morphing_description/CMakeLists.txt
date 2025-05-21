cmake_minimum_required(VERSION 3.8)
project(morphing_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ament build system
find_package(ament_cmake REQUIRED)
# Gazebo–ROS integration (for the plugins you load in SDF)
find_package(gazebo_ros REQUIRED)

# Install launch files and model resources
install(
  DIRECTORY launch model model/stl
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()