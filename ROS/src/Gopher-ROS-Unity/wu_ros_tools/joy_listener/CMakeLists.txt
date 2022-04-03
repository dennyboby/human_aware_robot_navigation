cmake_minimum_required(VERSION 2.8.3)
project(joy_listener)
find_package(catkin REQUIRED COMPONENTS
                rospy
                sensor_msgs
)
catkin_python_setup()
catkin_package(
    CATKIN_DEPENDS rospy sensor_msgs
)

if(CATKIN_ENABLE_TESTING)
  find_package(catkin REQUIRED COMPONENTS roslint)
  roslint_python()
  roslint_add_test()
endif()
