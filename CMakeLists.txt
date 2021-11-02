cmake_minimum_required(VERSION 3.5)
project(corner_detection)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
# find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(std_srvs REQUIRED)
# find_package(tf2_ros REQUIRED)
# find_package(tf2 REQUIRED)

find_package(pluginlib REQUIRED)
# find_package(filters REQUIRED)
# find_package(laser_filters REQUIRED)


include_directories(
  include
)

# add_library(interpolation_filter src/interpolation_filter.cpp)
# target_include_directories(interpolation_filter PUBLIC
# <BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
# <INSTALL_INTERFACE:include>)
# ament_target_dependencies(
#   interpolation_filter
#   filters
#   pluginlib
#   rclcpp
# )

# pluginlib_export_plugin_description_file(filters interpolation_filter.xml)

set(executable_name corner_detection)

# set(TEST_NAME test_scan_filter_chain)
# set(RESULT_FILENAME ${AMENT_TEST_RESULTS_DIR}/${PROJECT_NAME}/${TEST_NAME}.gtest.xml)
# ament_add_gtest_executable(${TEST_NAME} test/${TEST_NAME}.cpp)
# ament_target_dependencies(${TEST_NAME} filters pluginlib rclcpp)
# ament_add_test(
#     ${TEST_NAME}
#     COMMAND 
#         $<TARGET_FILE:${TEST_NAME}>
#         --ros-args --params-file ${PROJECT_SOURCE_DIR}/test/${TEST_NAME}.yaml
#         # --gtest_output=xml:${RESULT_FILENAME}
#     RESULT_FILE ${RESULT_FILENAME}
# )

add_executable(${executable_name}
  src/main.cpp
)

target_include_directories(${executable_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

set(dependencies
  rclcpp
  nav_msgs
  std_msgs
  geometry_msgs
  sensor_msgs
  # std_srvs
  # tf2_ros
  # tf2
  pluginlib
  # filters
  # laser_filters
  # laser_geometry
)

ament_target_dependencies(${executable_name}
  ${dependencies}
)

install(TARGETS ${executable_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install launch files.
install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
