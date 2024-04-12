


cmake_minimum_required(VERSION 3.8) # Use the higher version requirement
project(cpp_pubsub) # You can name your project appropriately

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi)
endif()

# Find packages for both OpenCV and ROS dependencies
find_package(OpenCV REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Python COMPONENTS Interpreter Development)

# Display OpenCV status
message(STATUS "OpenCV library status:")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG 0.8.0
)
FetchContent_GetProperties(yaml-cpp)

if(NOT yaml-cpp_POPULATED)
  message(STATUS "Fetching yaml-cpp...")
  FetchContent_Populate(yaml-cpp)
  add_subdirectory(${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR})
endif()

# Add executables for both the original blobDetection and the ROS version
add_executable(rosBlobDetection src/publisher_member_function.cpp)
ament_target_dependencies(rosBlobDetection rclcpp std_msgs)
ament_target_dependencies(rosBlobDetection rclcpp geometry_msgs)
target_link_libraries(rosBlobDetection ${OpenCV_LIBS} yaml-cpp::yaml-cpp)
# Installation rule for ROS executable
install(TARGETS
  rosBlobDetection
  DESTINATION lib/${PROJECT_NAME})


# Linting and testing configurations
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE) # Adjust based on your project needs
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

