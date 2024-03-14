cmake_minimum_required(VERSION 3.0.2)

project(mpc_controller)
find_package(catkin REQUIRED roscpp rospy std_msgs)
catkin_package()

catkin_install_python( PROGRAMS
src/mpc_controller.py
src/position_controller.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)