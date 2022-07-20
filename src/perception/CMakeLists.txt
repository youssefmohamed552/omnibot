cmake_minimum_required(VERSION 3.5)
project(perception)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

list(APPEND CMAKE_PREFIX_PATH "/usr/local/share/apriltag/cmake")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../lib/eigen/build")
message("PATH: ${CMAKE_PREFIX_PATH}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_srvs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(apriltag REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(omnibot_msgs REQUIRED)

find_package(OpenCV REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

add_library(
  perception STATIC
  src/perception/perception.cpp
)

set(Apriltag_LIB "${CMAKE_CURRENT_SOURCE_DIR}/lib/apriltag/build/libapriltag.so.3")

target_link_libraries(perception
  ${OpenCV_LIBRARIES}
  "/usr/local/lib/libapriltag.so.3"
)

ament_target_dependencies(perception
  camera_calibration_parsers
  cv_bridge
  image_transport
  rclcpp
  rclcpp_components
  sensor_msgs
  std_srvs
  stereo_msgs
  apriltag
  omnibot_msgs
)

add_executable(perception_node src/perception_node.cpp)
# target_include_directories(perception_node PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>)

target_link_libraries(perception_node perception)
ament_target_dependencies(perception_node rclcpp)
# add_dependencies(perception_node rclcpp)


install(TARGETS perception_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS perception
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS perception
  DESTINATION bin)

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

ament_export_dependencies(rclcpp)

ament_package()
