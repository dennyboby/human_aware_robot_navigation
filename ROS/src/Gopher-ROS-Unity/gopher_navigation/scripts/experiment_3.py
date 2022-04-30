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
        rospy.sleep(0.2)

    def goal_status_cb(self, data):
        status_list = data.status_list
        if len(status_list) != 0:
            status = status_list[-1].status
            if (status == 3):
                self.status == True
    
    def navigate(self):
        point1 = Point(-1.6808, -7.3111, 0.0)
        quat1 = Quaternion(0.0, 0.0, -0.7147419780269852, 0.6993882361364627)
        self.publish_goal(point1, quat1)
        print('Goal 1 Published')

        while not (self.status or rospy.is_shutdown()):
            rospy.sleep(0.2)

        rospy.sleep(5.0)

        point2 = Point(-1.6808, -7.3111, 0.0)
        quat2 = Quaternion(0.0, 0.0, -0.7147419780269852, 0.6993882361364627)
        self.publish_goal(point2, quat2)
        print('Goal 2 Published')

        while not (self.status or rospy.is_shutdown()):
            rospy.sleep(0.2)

        rospy.sleep(5.0)

        point3 = Point(-1.6808, -7.3111, 0.0)
        quat3 = Quaternion(0.0, 0.0, -0.7147419780269852, 0.6993882361364627)
        self.publish_goal(point3, quat3)
        print('Goal 3 Published')

        while not (self.status or rospy.is_shutdown()):
            rospy.sleep(0.2)

if __name__ == '__main__':
    rospy.init_node('experiment_3_node', anonymous=True)
    exp3 = Exp3()

    rospy.spin()