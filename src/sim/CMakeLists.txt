cmake_minimum_required(VERSION 3.8)
project(sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(
  include
  /usr/include/eigen3
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(omnibot_msgs REQUIRED)
find_package(Eigen3 REQUIRED PATHS /usr/share/eigen3/cmake)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_library(sim src/sim/sim.cpp)
ament_target_dependencies(sim
  rclcpp
  nav_msgs
  omnibot_msgs
)

add_executable(sim_node src/sim_node.cpp)

add_executable(cmd_node src/cmd_node.cpp)

add_executable(landmark_node src/landmark_publisher.cpp)

add_executable(stats_node src/stats_node.cpp)

target_link_libraries(sim_node sim)
ament_target_dependencies(sim_node 
  rclcpp
  nav_msgs
  geometry_msgs
  omnibot_msgs
)

ament_target_dependencies(cmd_node
  rclcpp
  geometry_msgs
)

ament_target_dependencies(landmark_node
  rclcpp
  omnibot_msgs
)

ament_target_dependencies(stats_node
  rclcpp
  nav_msgs
)
target_include_directories(sim_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(sim_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(cmd_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(landmark_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
target_compile_features(stats_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS sim_node cmd_node landmark_node stats_node
  DESTINATION lib/${PROJECT_NAME})


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
