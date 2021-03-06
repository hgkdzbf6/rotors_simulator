/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CONTROLLER_NODE_H
#define ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CONTROLLER_NODE_H

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <stdio.h>

#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <mav_msgs/Actuators.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Trigger.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include "rotors_control/common.h"
#include "rotors_control/roll_pitch_yawrate_thrust_controller.h"

namespace rotors_control {

class RollPitchYawrateThrustControllerNode {
 public:
  RollPitchYawrateThrustControllerNode();
  ~RollPitchYawrateThrustControllerNode();

  void InitializeParams();
  void Publish();

 private:

  RollPitchYawrateThrustController roll_pitch_yawrate_thrust_controller_;

  std::string namespace_;

  // subscribers
  ros::Subscriber cmd_multi_dof_joint_trajectory_sub_;
  ros::Subscriber cmd_roll_pitch_yawrate_thrust_sub_;
  ros::Subscriber odometry_sub_;
  // service client
  ros::ServiceClient svo_control_client_;
  ros::ServiceServer taking_off_server_;
  ros::Publisher motor_velocity_reference_pub_; 
  ros::Publisher twist_pub_;
  
  mav_msgs::EigenTrajectoryPointDeque commands_;
  std::deque<ros::Duration> command_waiting_times_;
  double take_off_height_;

  // timer
  ros::Timer command_timer_;
  ros::Timer timer_;
  EigenOdometry odometry_;

  void RollPitchYawrateThrustCallback(
      const mav_msgs::RollPitchYawrateThrustConstPtr& roll_pitch_yawrate_thrust_reference_msg);
  void MultiDofJointTrajectoryCallback(
      const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& trajectory_reference_msg);
  
  bool TakingoffCallback(
  		std_srvs::Trigger::Request &req,
  		std_srvs::Trigger::Response &res);
  void TimedCommandCallback(const ros::TimerEvent& e);
  void TimerCallback(const ros::TimerEvent& e);
  
  void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
};
}

#endif // ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CONTROLLER_NODE_H
