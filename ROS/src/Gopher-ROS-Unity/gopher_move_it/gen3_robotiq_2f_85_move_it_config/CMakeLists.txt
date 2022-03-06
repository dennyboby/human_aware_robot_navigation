cmake_minimum_required(VERSION 2.8.3)
project(gen3_robotiq_2f_85_move_it_config)

#find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIRS})

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


#add_executable(robot_model_and_robot_state_tutorial src/robot_model_and_robot_state_tutorial.cpp)
#target_link_libraries(robot_model_and_robot_state_tutorial ${catkin_LIBRARIES} ${Boost_LIBRARIES})

#install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
#install(TARGETS robot_model_and_robot_state_tutorial DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

