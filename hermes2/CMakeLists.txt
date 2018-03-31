cmake_minimum_required(VERSION 3.5)
project(ceres2018)

find_package(ament_cmake REQUIRED)
find_package(rclc REQUIRED)
find_package(ceres2018_msgs REQUIRED)

add_executable(ceres2018 main.c)
ament_target_dependencies(ceres2018 rclc ceres2018_msgs)

install(TARGETS
  ceres2018
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
