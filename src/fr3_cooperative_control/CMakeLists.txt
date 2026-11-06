cmake_minimum_required(VERSION 3.8)
project(fr3_cooperative_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# --- Mevcut koordinator (degistirilmedi) ---
add_executable(dual_arm_coordinator src/dual_arm_coordinator.cpp)
target_include_directories(dual_arm_coordinator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(dual_arm_coordinator PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  dual_arm_coordinator
  "rclcpp"
  "moveit_ros_planning_interface"
  "geometry_msgs"
  "tf2_geometry_msgs"
)

# --- Asama 1: uzanma testi (kutu yok, gripper yok) ---
add_executable(dual_arm_reach_test src/dual_arm_reach_test.cpp)
target_include_directories(dual_arm_reach_test PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(dual_arm_reach_test PUBLIC c_std_99 cxx_std_17)
ament_target_dependencies(
  dual_arm_reach_test
  "rclcpp"
  "moveit_ros_planning_interface"
  "moveit_msgs"
  "geometry_msgs"
  "tf2_geometry_msgs"
)

install(TARGETS dual_arm_coordinator dual_arm_reach_test
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

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
