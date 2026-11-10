cmake_minimum_required(VERSION 3.8)
project(topping_delta_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES NOTICE LICENSE
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
