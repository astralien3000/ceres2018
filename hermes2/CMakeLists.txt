cmake_minimum_required(VERSION 3.5)
project(ceres2018)

find_package(ament_cmake REQUIRED)
find_package(rclc REQUIRED)
find_package(ceres2018_msgs REQUIRED)

set(sources
  main.c
  motor.h
  motor.c
  encoder.h
  encoder.c
  scheduler.h
  scheduler.c
  speed_motor.h
  )

riot_add_config(ceres2018 "FEATURES_REQUIRED += periph_pwm")
riot_add_config(ceres2018 "USEMODULE += printf_float")

add_executable(ceres2018 ${sources})
ament_target_dependencies(ceres2018 rclc ceres2018_msgs)

install(TARGETS
  ceres2018
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
