cmake_minimum_required(VERSION 3.8)
project(caret_sample)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(sample_0 src/sample_0.cpp include/caret_sample/node.hpp)
ament_target_dependencies(sample_0  "rclcpp" "std_msgs")
target_include_directories(sample_0 PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(sample_sub3pub1 src/sample_sub3pub1.cpp include/caret_sample/node.hpp)
ament_target_dependencies(sample_sub3pub1  "rclcpp" "std_msgs")
target_include_directories(sample_sub3pub1 PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


install(TARGETS
  sample_0 sample_sub3pub1
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
