cmake_minimum_required(VERSION 3.5)
project(volcanibot_controller)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

install(PROGRAMS
  src/volcanibot_controller/trajectory_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)
