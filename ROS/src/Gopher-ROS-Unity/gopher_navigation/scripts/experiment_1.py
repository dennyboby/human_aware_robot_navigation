#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion

if __name__ == '__main__':
    rospy.init_node('experiment_1_node', anonymous=True)
    pose_pub = rospy.Publisher('/gopher_presence/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(0.5) # wait for the publisher to be ready
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = 'map'
    goal.pose.position = Point(-1.6808, -7.3111, 0.0)
    goal.pose.orientation = Quaternion(0.0, 0.0, -0.7147419780269852, 0.6993882361364627)
    pose_pub.publish(goal)
    print('Goal Published')
    rospy.spin()