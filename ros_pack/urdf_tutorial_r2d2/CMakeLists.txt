cmake_minimum_required(VERSION 3.5)
project(urdf_tutorial_r2d2)

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
