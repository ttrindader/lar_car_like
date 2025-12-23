cmake_minimum_required(VERSION 3.0.2)
project(car_like_description)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY launch urdf meshes rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

