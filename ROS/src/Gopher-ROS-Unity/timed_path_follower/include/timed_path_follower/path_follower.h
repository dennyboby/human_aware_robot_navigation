/*******************************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015 Marina Kollmitz
 *  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Marina Kollmitz
 ******************************************************************************/

#ifndef SIMPLE_CONTROLLER_HEADER_H
#define SIMPLE_CONTROLLER_HEADER_H

#include <ros/ros.h>
#include <angles/angles.h>
#include <tf/tf.h>
#include <dynamic_reconfigure/server.h>
#include <pluginlib/class_list_macros.hpp>
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Empty.h>
#include <timed_path_follower/PathExecuterConfig.h>

namespace path_executer
{

/**
 * @brief The PathFollower class provides a local planner for differential drive
 *        robots to follow dynamically feasible, time dependent global paths.
 *
 * The given global path has to be dynamically feasible for differential drive
 * robots so that the control velocities can be recovered from the path and the
 * associated time stamps. Unfortunately, it is not sufficient to use the control
 * velocities directly because of localization and motion uncertainties.
 * Therefore, the velocity commands are calculated based on a feedback motion
 * controller for differential drive robots as suggested by Siegwart et al.
 * (Siegwart , R. ; Nourbakhsh , I. R. ; Scaramuzza, D.:
 * Introduction to Autonomous Mobile Robots. MIT Press, 2011).
 *
 * The planner is written as a local planner plugin for the ROS navigation stack
 * and adheres to the nav_core::BaseLocalPlanner interface.
 */
class PathFollower : public nav_core::BaseLocalPlanner
{
public:

  /**
   * @brief constructor
   */
  PathFollower();

  /**
   * @brief initialize the planner
   *
   * @param name the name to give this instance of the trajectory planner
   * @param tf transform listener
   * @param costmap_ros the (local) cost map that defines environment constraints
   *        for navigation
   */
  void initialize(std::string name, tf2_ros::Buffer* tf,
                  costmap_2d::Costmap2DROS* costmap_ros);

  /**
   * @brief compute the velocity commands to execute the timed plan
   *
   * Given the current position, orientation, and velocity of the robot,
   * compute velocity commands to send to the base. The current position of the
   * robot is compared to the waypoint of the given global path that corresponds
   * to the current time. The difference between both poses is used to generate
   * the control velocity, following the feedback motion controller for
   * differential drive robots suggested by Siegwart et al.
   *
   * @param[out] cmd_vel will be filled with the control velocity command
   * @return true if a valid velocity command was found, false otherwise
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

  /**
   * @brief set global plan
   *
   * @param global_plan the (dynamically feasible and time dependent) global plan
   * @return true if global plan was successfully set, false otherwise
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan);

  /**
   * @brief compute velocities along a (dynamically feasible, time dependent) plan
   *
   * given the waypoints of the plan and their associated time stamps, compute the necessary
   * control velocities to follow the plan (assuming differential drive kinematics)
   *
   * @param timed_plan the (dynamically feasible and time dependent) navigation plan
   * @return list with associated waypoint velocities
   */
  std::vector<geometry_msgs::Twist> computeWaypointVelocities(const std::vector<geometry_msgs::PoseStamped> &timed_plan);

  /**
   * @brief find the waypoint on the global path that corresponds to the requested time
   *
   * @param time requested time
   * @param[out] waypoint corresponding waypoint
   * @param[out] waypoint_vel corresponding velocity
   * @return true if the requested time is represented by the path, false if it is
   *         outside the path range
   */
  bool findWaypointAtTime(ros::Time time, geometry_msgs::PoseStamped& waypoint,
                          geometry_msgs::Twist &waypoint_vel);

  /**
   * @brief check if the robot has reached the goal
   *
   * @return whether the goal is reached
   */
  bool isGoalReached()
  {
    return goal_reached_;
  }

  /**
   * @brief dynamic reconfigure callback
   *
   * @param config dynamic reconfigure configuration with controller velocity
   *        limits and gains
   * @param level level
   */
  void reconfigureCallback(path_executer::PathExecuterConfig &config,
                            uint32_t level);

private:

  // controller preferences
  double max_vel_x_; ///< maximum forward velocity
  double max_vel_phi_; ///< maximum rotation velocity
  double xy_goal_tolerance_; ///< euclidian goal distance tolerance
  double yaw_goal_tolerance_; ///< rotational goal distance tolerance
  bool allow_backwards_; ///< whether backwards motion is allowed for the controller

  //local planner status flags
  bool initialized_; ///< whether the planner is initialized
  bool goal_reached_; ///< whether the robot has reached the goal
  bool replanning_requested_; ///< whether replanning for the global path was requested

  //dynamically feasible, time dependent global plan
  std::vector<geometry_msgs::PoseStamped> global_plan_waypoints_; ///< global plan waypoints
  std::vector<geometry_msgs::Twist> global_plan_velocities; ///< global plan velocities
  geometry_msgs::PoseStamped goal_; ///< navigation goal

  //controller gains:
  double k_rho_; ///< controller gain for euclidian distance
  double k_alpha_; ///< controller gain for heading difference to face path
  double k_beta_; ///< controller gain for desired orientation

  //ros related stuff
  costmap_2d::Costmap2DROS* costmap_ros_; ///< cost map with navigation constraints
  ros::ServiceClient replan_client_; ///< service client to make a replanning request to the global planner
  tf2_ros::Buffer* tf_; ///< tf2 buffer
  dynamic_reconfigure::Server<path_executer::PathExecuterConfig> *dsrv_; ///< dynamic reconfigure server
  ros::Publisher current_waypoint_pub_; ///< ros publisher to visualize the currently scheduled waypoint

};
} //namespace path_executer

#endif
