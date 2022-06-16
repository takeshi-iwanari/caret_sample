cmake_minimum_required(VERSION 3.8)
project(caret_sample)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

function(create_sample NAME)
  add_executable("${NAME}" "src/${NAME}.cpp" include/caret_sample/node.hpp)
  ament_target_dependencies("${NAME}"  "rclcpp" "std_msgs")
  target_include_directories("${NAME}" PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  install(TARGETS "${NAME}" DESTINATION lib/${PROJECT_NAME})
endfunction()

create_sample("sample_straight")
create_sample("sample_sub3pub1")
create_sample("sample_store")
create_sample("sample_huge_latency")
create_sample("sample_store_timer_1")
create_sample("sample_store_timer_2")
create_sample("sample_sub1pubtimer3")
create_sample("sample_feedback")

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
