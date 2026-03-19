cmake_minimum_required(VERSION 3.8)
project(robot_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf/    DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY meshes/  DESTINATION share/${PROJECT_NAME}/meshes)
install(DIRECTORY rviz/    DESTINATION share/${PROJECT_NAME}/rviz)
install(DIRECTORY launch/  DESTINATION share/${PROJECT_NAME}/launch)

ament_package()
