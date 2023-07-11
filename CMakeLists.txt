cmake_minimum_required(VERSION 3.5)
project(final_assessment)

find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(navigation2 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(navigation2 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)


add_executable(fa_nav src/fa_nav.cpp)

ament_target_dependencies(fa_nav
  "rclcpp" 
  "std_msgs"
  "rclcpp_action"
  "nav2_msgs"
  "tf2_ros"
  "nav_msgs"
  "geometry_msgs"
  "nav2_util"
  )

target_include_directories(fa_nav PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS 
  fa_nav
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  configs
  worlds
  robots
  components
  models
  DESTINATION share/${PROJECT_NAME})

ament_package()
