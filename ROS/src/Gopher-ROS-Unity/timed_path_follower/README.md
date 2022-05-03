# timed_path_follower

# Overview #

The timed_path_follower package provides a local planner plugin for the ROS navigation stack. 

The local planner implements a controller for robots with differential drive constraints to follow time dependent, dynamically feasible navigation paths.  It is based on a feedback motion controller for differential drive robots as suggested by Siegwart et al. (Siegwart, R. ; Nourbakhsh, I. R. ; Scaramuzza, D.: Introduction to Autonomous Mobile Robots. MIT Press, 2011). Note that the local planner is only concerned with following a given robot trajectory as closely as possible, it does not perform any collision checking. Therefore, the global planner frequency should be high enough to ensure obstacle avoidance. The local planner frequency should be large enough, at least 10 Hz, because it might become unstable otherwise. 

# Configuration for ROS navigation stack #

To use the timed path follower with the ROS navigation stack you need to set the following parameter for move_base: 
        
    move_base/base_local_planner: path_executer/PathFollower

You can also use the configuration file (timed_path_follower/config/local_planner_params.yaml) with move base to set all parameters for using the path follower with move_base from a launch file:

    <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">   
        <rosparam file="$(find timed_path_follower)/config/local_planner_params.yaml" command="load" />
    </node>

# ROS Parameters: #

The following parameters configure the behavior of the path follower:

prefix for all ROS params: move_base/PathFollower/ 

- allow_backwards --> to allow backwards motion when trying to follow a trajectory.
- xy_goal_tolerance --> euclidian goal distance threshold in meters
- yaw_goal_tolerance --> rotational goal distance threshold in radians

The controller gains and velocity bounds for the path follower can be adjusted using dynamic reconfigure (move_base/PathFollower prefix).
