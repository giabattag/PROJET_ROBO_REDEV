cmake_minimum_required(VERSION 3.5)
project(projet_robo_redev)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

install(
  DIRECTORY images launch meshes rviz urdf world
  DESTINATION share/${PROJECT_NAME}
)



ament_package()
