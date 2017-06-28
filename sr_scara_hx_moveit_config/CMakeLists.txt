cmake_minimum_required(VERSION 2.8.3)
project(sr_scara_hx_moveit_config)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  geometry_msgs
  moveit_msgs
  roscpp
  rospy
  sensor_msgs
  std_srvs
  std_msgs
  tf
  trajectory_msgs
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    control_msgs
    geometry_msgs
    moveit_msgs
    roscpp
    rospy
    sensor_msgs
    std_srvs
    std_msgs
    tf
    trajectory_msgs
)

catkin_python_setup()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
