cmake_minimum_required(VERSION 2.8.3)
project(easy_markers)
find_package(catkin REQUIRED COMPONENTS
                interactive_markers
                roslib
                geometry_msgs
                visualization_msgs
                rospy
                tf
)
catkin_python_setup()
catkin_package(
    CATKIN_DEPENDS geometry_msgs interactive_markers rospy tf visualization_msgs
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_python()
  roslint_python(scripts)
  roslint_add_test()
endif()

catkin_install_python(PROGRAMS scripts/interactive_marker_demo.py scripts/std_marker_demo.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
