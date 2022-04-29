import rospy
from people_msgs import PeoplePrediction

def people_callback():
    pass

def robot_callback():
    pass

def subscribers():
    rospy.init_node('social_dist', anonymous=True)

    rospy.Subscriber('people_prediction', PeoplePrediction, people_callback)

    rospy.Subscriber('gopher_presence/robot_pose', PeoplePrediction, robot_callback)

if __name__ == '__main__':
    subscribers()