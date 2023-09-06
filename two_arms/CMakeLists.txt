cmake_minimum_required(VERSION 2.8.3)
project(two_arms)

find_package(catkin REQUIRED COMPONENTS
  rospy
)


catkin_package()



## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/motionGroupPosController.py
  scripts/motionGroupPosControllerCopy.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)



## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


