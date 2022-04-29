#!/usr/bin/env python

import rospy
import math
import numpy as np
from people_msgs.msg import People
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray

class SocialDist:
    def __init__(self):
        self.person_position = np.array([math.inf, math.inf, math.inf])
        self.robot_position = np.array([math.inf, math.inf, math.inf])
        self.min_dist = math.inf
        self.status = 0
        self.subscribers()

    def goal_status_cb(self, data):
        status_list = data.status_list
        if len(status_list) != 0:
            status = status_list[-1].status
            if (status == 3) and (self.status != 3):
                print("Minimum Distance: ", self.min_dist)
            elif (status == 3) and (self.status == 3):
                pass
            else:
                self.calculate_min_dist()
                # print("Robot Moving: ", self.min_dist)
            self.status = status

    def people_callback(self, data):
        people_arr = data.people
        for person in people_arr:
            pos = person.position
            self.person_position = np.array([pos.x, pos.y, pos.z])

    def robot_callback(self, data):
        robot_pos = data.pose.position
        self.robot_position = np.array([robot_pos.x, robot_pos.y, robot_pos.z])

    def calculate_min_dist(self):
        dist = np.linalg.norm(self.robot_position - self.person_position)
        if dist < self.min_dist:
            self.min_dist = dist
            # print("Update Distance: ", self.min_dist)

    def subscribers(self):
        rospy.init_node('social_dist', anonymous=True)

        rospy.Subscriber('people', People, self.people_callback)
        rospy.Subscriber('gopher_presence/robot_pose', PoseStamped, self.robot_callback)
        rospy.Subscriber('gopher_presence/move_base/status', GoalStatusArray, self.goal_status_cb)
        rospy.spin()

if __name__ == '__main__':
    SocialDist()