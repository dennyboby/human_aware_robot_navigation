#!/usr/bin/env python

import rospy
from math import atan2, cos, sin, pi
import numpy as np
from angles import normalize_angle, shortest_angular_distance
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Twist, PoseStamped, Pose
from freight_navigation.msg import NavIntent

PREDEFINED_PATH = [
    {'start_end':[1,2],
        'path':[[6.209, -13.884],[6.209, -11.545],[2.293, -11.545]]},
    {'start_end':[1,3],
        'path':[[6.209, -13.884],[6.209, -7.378],[14.504, -7.378]]},
    {'start_end':[2,3],
        'path':[[2.293, -11.545],[6.209, -11.545],[6.209, -7.378],[14.504, -7.378]]}
]
ROOM_DETECTION_RADIUS = 0.3
POSE_THRESHOLD = 0.1
ORIENTATION_THRESHOLD = 0.00872665
ANGULAR_VELOCITY = 0.1
LINEAR_VELOCITY = 0.15
STOP_VELOCITY = 0.0
HOME_ROOM = 1
KP = 0.1 
KI = 0.01
KD = 0.1

class Navigation():
    def __init__(self):
        self.rate = rospy.Rate(60)
        self.robot_pose = Pose()
        self.prev_goal = None
        self.current_pose_sub = rospy.Subscriber('/model_pose',PoseStamped ,self.current_pose_cb)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist ,queue_size=1)
        self.navigation_sub = rospy.Subscriber('/navigation_goal', NavIntent, self.navigation_request_cb)

    def achieve_orientation(self, desired_angle):
        goal_angle = normalize_angle(desired_angle)
        current_angle = normalize_angle(euler_from_quaternion([self.robot_pose.orientation.x,
                            self.robot_pose.orientation.y, self.robot_pose.orientation.z,
                            self.robot_pose.orientation.w])[2])
        # print("Angles are {} and {}".format(goal_angle, current_angle))
        # print("Shortest angle is {}".format(shortest_angular_distance(current_angle, goal_angle)))
        while abs(shortest_angular_distance(current_angle, goal_angle)) >= ORIENTATION_THRESHOLD and not rospy.is_shutdown():
            if shortest_angular_distance(current_angle, goal_angle) > 0:
                msg = Twist()
                msg.angular.z = ANGULAR_VELOCITY
                self.cmd_vel_pub.publish(msg)
            else:
                msg = Twist()
                msg.angular.z = -ANGULAR_VELOCITY
                self.cmd_vel_pub.publish(msg)
            current_angle = normalize_angle(euler_from_quaternion([self.robot_pose.orientation.x,
                                self.robot_pose.orientation.y, self.robot_pose.orientation.z,
                                self.robot_pose.orientation.w])[2])
            self.rate.sleep()
        msg = Twist()
        msg.angular.z = STOP_VELOCITY
        self.cmd_vel_pub.publish(msg)
        return True

    def steering_angle(self, goal):
        v = [cos(pi), sin(pi), 0]
        rg = [goal[0]-self.position[0], goal[1]-self.position[1], 0]
        prodNorm = np.linalg.norm(v)*np.linalg.norm(rg)
        cosPhi = np.dot(v, rg)/prodNorm
        sinPhi = np.cross(v, rg)[2]/prodNorm
        phi = atan2(sinPhi, cosPhi)
        return min(max(phi, -pi/4.1), pi/4.1)

    def compute_velocity(self, goal):
        if self.prev_goal != goal:
            self.E = 0  #Cummulative error
            self.old_e = 0 # Previous error
            self.prev_goal = goal
            
        #Difference in x and y
        d_x = goal[0] - self.robot_pose.position.x
        d_y = goal[1] - self.robot_pose.position.y

        #Angle from robot to goal
        g_theta = atan2(d_y, d_x)

        #Error between the goal angle and robot angle
        alpha = g_theta - normalize_angle(euler_from_quaternion([self.robot_pose.orientation.x,
                            self.robot_pose.orientation.y, self.robot_pose.orientation.z,
                            self.robot_pose.orientation.w])[2])
        #alpha = g_theta - math.radians(90)
        e = atan2(sin(alpha), cos(alpha))

        e_P = e
        e_I = self.E + e
        e_D = e - self.old_e

        # This PID controller only calculates the angular velocity with constant speed of v
        # The value of v can be specified by giving in parameter or using the pre-defined value defined above.
        w = KP*e_P + KI*e_I + KD*e_D

        w = atan2(sin(w), cos(w))

        self.E = self.E + e
        self.old_e = e
        v = LINEAR_VELOCITY

        return v, w

    def achieve_position(self, position):
        eucd_dist = np.linalg.norm(np.array(position) - np.array((self.robot_pose.position.x, self.robot_pose.position.y)))
        while eucd_dist > POSE_THRESHOLD and not rospy.is_shutdown():
            msg = Twist()
            msg.linear.x, msg.angular.z = self.compute_velocity(position)
            self.cmd_vel_pub.publish(msg)
            eucd_dist = np.linalg.norm(np.array(position) - np.array((self.robot_pose.position.x, self.robot_pose.position.y)))
            # print(position[0], position[1], self.robot_pose.position.x, self.robot_pose.position.y)
            # print(eucd_dist)
            self.rate.sleep()
        msg = Twist()
        msg.linear.x = STOP_VELOCITY
        self.cmd_vel_pub.publish(msg)
        return True

    def get_current_room(self):
        for room in PREDEFINED_PATH:
            x, y = self.robot_pose.position.x, self.robot_pose.position.y
            if np.linalg.norm(np.array(room['path'][0]) - np.array([x, y])) < ROOM_DETECTION_RADIUS:
                return room['start_end'][0]
            elif np.linalg.norm(np.array(room['path'][-1]) - np.array([x, y])) < ROOM_DETECTION_RADIUS:
                return room['start_end'][1]
        return None
            
    def get_path(self, start, end):
        for room in PREDEFINED_PATH:
            if room['start_end'] == [start, end]:
                return room['path']
            elif room['start_end'] == [end, start]:
                return room['path'][::-1]
        return None

    def get_angle(self, position1, position2):
        x = position2[0] - position1[0]
        y = position2[1] - position1[1]
        angle = atan2(y,x)
        return angle

    def execute_path(self, path):
        for index, position in enumerate(path):
            if index != 0:
                angle = self.get_angle(path[index-1], position)
                # print(angle)
                self.achieve_orientation(angle)
                self.achieve_position(position)
        return True

    def process_navigation_request(self):
        if self.intent == "move":
            for goal in self.goals:
                current_room = self.get_current_room()
                if current_room == None:
                    print("No room detected")
                    return False
                path = self.get_path(current_room, goal)
                if path == None:
                    print("No path found")
                    return False
                self.execute_path(path)
                print("Reached room {}".format(self.get_current_room()))
        elif self.intent == "exit":
            current_room = self.get_current_room()
            if current_room == None:
                print("No room detected")
                return False
            path = self.get_path(current_room, HOME_ROOM)
            if path == None:
                print("No path found")
                return False
            self.execute_path(path)
            print("Reached home position")
        else:
            print("Invalid intent for navigation")

    def current_pose_cb(self, msg):
        self.robot_pose = msg.pose

    def navigation_request_cb(self, msg):
        print("Navigation request received")
        self.intent = msg.intent
        self.goals = msg.goals
        self.process_navigation_request()

if __name__ == '__main__':
    rospy.init_node('navigation_node')
    navigation = Navigation()
    rospy.spin()
