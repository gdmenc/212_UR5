cmake_minimum_required(VERSION 3.0.2)
project(dual_ur5_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  geometry_msgs
  rospy
  tf
  xacro
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/spawn_scene.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  models
  worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
