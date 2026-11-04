cmake_minimum_required(VERSION 3.8)
project(humanoid_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch rviz meshes
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/robot_description_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
