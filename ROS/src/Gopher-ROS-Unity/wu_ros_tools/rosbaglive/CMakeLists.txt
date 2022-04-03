cmake_minimum_required(VERSION 2.8.3)
project(rosbaglive)
find_package(catkin REQUIRED COMPONENTS rospy rosbag)
catkin_package()

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_python(scripts)
  roslint_add_test()
endif()

catkin_install_python(PROGRAMS scripts/play.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
