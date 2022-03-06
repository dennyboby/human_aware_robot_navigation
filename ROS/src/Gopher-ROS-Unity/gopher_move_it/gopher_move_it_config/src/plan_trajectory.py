#!/usr/bin/env python

import rospy

import sys
import copy
import math
import moveit_commander

from sensor_msgs.msg import JointState
from moveit_msgs.msg import RobotState
from gopher_move_it_config.srv import PlanTrajectory, PlanTrajectoryRequest, PlanTrajectoryResponse

# Between Melodic and Noetic, the return type of plan() changed. 
# moveit_commander has no __version__ variable, so checking the python version as a proxy
if sys.version_info >= (3, 0):
    def planCompat(plan):
        return plan[1]
else:
    def planCompat(plan):
        return plan


def plan_trajectory(move_group, destination_pose, start_joint_angles): 
    """ Given the start angles of the robot, 
        plan a trajectory that ends at the destination pose.
    """ 
    current_joint_state = JointState()
    current_joint_state.name = ["kinova/joint_"+str(i+1) for i in range(len(start_joint_angles))]
    current_joint_state.position = start_joint_angles

    moveit_robot_state = RobotState()
    moveit_robot_state.joint_state = current_joint_state

    move_group.set_start_state(moveit_robot_state)
    move_group.set_pose_target(destination_pose)
    plan = move_group.plan()

    if not plan:
        exception_str = """
            Trajectory could not be planned for a destination of {} with starting joint angles {}.
            Please make sure target and destination are reachable by the robot.
        """.format(destination_pose, destination_pose)
        raise Exception(exception_str)

    return planCompat(plan)


def plan(req):
    response = PlanTrajectoryResponse()

    # Initialize moveit
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    current_robot_joint_configuration = req.joints.joints

    # Pre grasp - position gripper directly above target object
    pre_grasp_pose = plan_trajectory(move_group, req.target, 
                                     current_robot_joint_configuration)
    
    # If the trajectory has no points, planning has failed
    if not pre_grasp_pose.joint_trajectory.points:
        return response
    else:
        response.trajectory = pre_grasp_pose

    move_group.clear_pose_targets()

    return response


def moveit_server():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('kinova_moveit')

    s = rospy.Service('move_group/plan_trajectory', PlanTrajectory, plan)
    print("Ready to plan")
    rospy.spin()


if __name__ == "__main__":
    moveit_server()
