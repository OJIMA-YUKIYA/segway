project(mywork)

find_package(
    catkin REQUIRED COMPONENTS
    std_msgs
    rospy
)


catkin_package()

catkin_install_python(PROGRAMS ./obstacleDetection.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
