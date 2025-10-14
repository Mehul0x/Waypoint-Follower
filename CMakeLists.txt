cmake_minimum_required(VERSION 3.8)
project(waypoint_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# === Executable for Trajectory Generator ===
add_executable(traj_gen_vis_node src/traj_gen_vis_node.cpp)
target_include_directories(traj_gen_vis_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(traj_gen_vis_node PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  traj_gen_vis_node
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "visualization_msgs"
  "tf2"
  "tf2_geometry_msgs"
)

# === Executable for Tracking Controller ===
add_executable(tracking_controller_node src/tracking_controller_node.cpp)
target_include_directories(tracking_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
ament_target_dependencies(
  tracking_controller_node
  "rclcpp"
  "nav_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_geometry_msgs"
)

target_link_libraries(tracking_controller_node
  tf2::tf2
  tf2_geometry_msgs::tf2_geometry_msgs
)

# === Install Rules ===
install(TARGETS
  traj_gen_vis_node
  tracking_controller_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install rviz config files
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/
)

# ... (other cmake content)

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