cmake_minimum_required(VERSION 2.8.3)
project(sr_scara_hx_moveit_config)

find_package(catkin REQUIRED
	roscpp
	rospy
	actionlib
	shape_msgs
	std_srvs
	trajectory_msgs
	sensor_msgs
	diagnostic_msgs
	moveit_msgs
	control_msgs
	tf
	geometry_msgs
	manipulation_msgs
	)

catkin_package()

catkin_python_setup()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
