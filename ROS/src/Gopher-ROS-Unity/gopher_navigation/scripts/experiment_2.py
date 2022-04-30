import rospy
from geometry_msgs.msg import PoseStamped, Point, Quaternion

if __name__ == '__main__':
    rospy.init_node('experiment_1_node', anonymous=True)
    pose_pub = rospy.Publisher('/gopher_presence/move_base_simple/goal', PoseStamped, queue_size=10)
    rospy.sleep(0.5) # wait for the publisher to be ready
    goal = PoseStamped()
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = 'map'
    goal.pose.position = Point(-11.5840, -5.8116, 0.0)
    goal.pose.orientation = Quaternion(0.0, 0.0, 0.9999932924620973, 0.003662653520906675)
    pose_pub.publish(goal)
    print('Goal Published')
    rospy.spin()