#!/usr/bin/env python

import rospy
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, PoseStamped, Pose
from actionlib_msgs.msg import GoalStatusArray

class Path():
    def __init__(self):
        self.rate = rospy.Rate(60)
        self.robot_pose_current = None
        self.robot_pose_previous = None
        self.goal_pose = None
        self.status = None
        self.path_length = 0
        self.current_pose_sub = rospy.Subscriber('/gopher_presence/robot_pose',PoseStamped ,self.current_pose_cb)
        self.goal_status_sub = rospy.Subscriber('/gopher_presence/move_base/status',GoalStatusArray,self.goal_status_cb)
        # self.current_goal_sub = rospy.Subscriber('/gopher_presence/move_base/current_goal',PoseStamped ,self.current_goal_cb)

    def get_distance(self, pose1, pose2):
        return np.sqrt(np.sum((pose1.position.x - pose2.position.x)**2 + (pose1.position.y - pose2.position.y)**2))
    
    def current_pose_cb(self,msg):
        # print("Receiving current pose")
        self.robot_pose_current = msg.pose

    def goal_status_cb(self,msg):
        # print("Receiving goal status")
        if len(msg.status_list) > 0:
            if msg.status_list[-1].status == 3:
                if self.status == 1:
                    print("Goal reached. The total path length is: ", self.path_length)
                self.status = 3
            elif msg.status_list[-1].status == 1:
                if self.status == 3 or self.status == None:
                    print("Goal is active.")
                    self.path_length = 0
                    self.robot_pose_previous = self.robot_pose_current
                self.status = 1
            if self.status == 1:
                # print("The current path length is: ", self.path_length)
                self.path_length += self.get_distance(self.robot_pose_current, self.robot_pose_previous)
                self.robot_pose_previous = self.robot_pose_current
    
    # def current_goal_cb(self,msg):
    #     self.goal_pose = msg.pose

if __name__ == '__main__':
    rospy.init_node('path_length_node')
    navigation = Path()
    rospy.spin()