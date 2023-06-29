cmake_minimum_required(VERSION 3.5)
project(final_assessment)

find_package(ament_cmake REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(navigation2 REQUIRED)

install(DIRECTORY launch
  configs
  maps
  worlds
  robots
  components
  models
  rviz
  DESTINATION share/${PROJECT_NAME})

ament_package()
