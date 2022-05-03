#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from actionlib_msgs.msg import GoalStatusArray

class Exp3:
    def __init__(self):
        self.pose_pub = rospy.Publisher('/gopher_presence/move_base_simple/goal', PoseStamped, queue_size=10)
        self.goal_sub = rospy.Subscriber('/gopher_presence/move_base/status',GoalStatusArray,self.goal_status_cb)
        rospy.sleep(0.5)
        self.status = False
        self.navigate()

    def publish_goal(self, point, quat):
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = 'map'
        goal.pose.position = point
        goal.pose.orientation = quat
        self.pose_pub.publish(goal)
        self.status = False
        # print('Status in publish: ', self.status)
        print('Goal Published')
        rospy.sleep(0.2)

    def goal_status_cb(self, data):
        status_list = data.status_list
        # print('Callback called.')
        if len(status_list) != 0:
            status = status_list[-1].status
            if (status == 3):
                # print('Goal reached')
                self.status = True
                # print('Status in callback: ', self.status)
    
    def navigate(self):
        point1 = Point(-1.2590, -2.55271, 0.0)
        quat1 = Quaternion(0.0, 0.0, -0.702835905578862, 0.7113520154108941)
        self.publish_goal(point1, quat1)
        print('Goal 1 Published')

        while not (self.status or rospy.is_shutdown()):
            # print('Status: ', self.status)
            rospy.sleep(0.2)

        rospy.sleep(5.0)

        point2 = Point(-10.714281, -8.329421, 0.0)
        quat2 = Quaternion(0.0, 0.0, -0.9119776654713311, 0.4102398538434083)
        self.publish_goal(point2, quat2)
        print('Goal 2 Published')

        while not (self.status or rospy.is_shutdown()):
            rospy.sleep(0.2)

        rospy.sleep(5.0)

        point3 = Point(7.0304985, -3.261139, 0.0)
        quat3 = Quaternion(0.0, 0.0, 0.6998626562032497, 0.7142774408114342)
        self.publish_goal(point3, quat3)
        print('Goal 3 Published')

        while not (self.status or rospy.is_shutdown()):
            rospy.sleep(0.2)

if __name__ == '__main__':
    rospy.init_node('experiment_3_node', anonymous=True)
    exp3 = Exp3()

    rospy.spin()