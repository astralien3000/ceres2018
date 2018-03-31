cmake_minimum_required(VERSION 3.5)
project(ceres2018_client)

find_package(ament_cmake REQUIRED)
find_package(rclc REQUIRED)
find_package(ceres2018_msgs REQUIRED)

add_executable(ceres2018_client main.c)
ament_target_dependencies(ceres2018_client rclc ceres2018_msgs)

install(TARGETS
  ceres2018_client
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
