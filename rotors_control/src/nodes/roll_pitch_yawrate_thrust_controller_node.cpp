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

#include "roll_pitch_yawrate_thrust_controller_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

RollPitchYawrateThrustControllerNode::RollPitchYawrateThrustControllerNode() {
  InitializeParams();

  ros::NodeHandle nh;

  cmd_roll_pitch_yawrate_thrust_sub_ = nh.subscribe(kDefaultCommandRollPitchYawrateThrustTopic, 1,
                                     &RollPitchYawrateThrustControllerNode::RollPitchYawrateThrustCallback, this);
  
  cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &RollPitchYawrateThrustControllerNode::MultiDofJointTrajectoryCallback, this);
  
  odometry_sub_ = nh.subscribe(kDefaultOdometryTopic, 1,
                               &RollPitchYawrateThrustControllerNode::OdometryCallback, this);

  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(
      kDefaultCommandMotorSpeedTopic, 1);  
      
  command_timer_ = nh.createTimer(ros::Duration(0), &RollPitchYawrateThrustControllerNode::TimedCommandCallback, this,
                                  true, false);

  taking_off_server_ = nh.advertiseService("taking_off",
		  &RollPitchYawrateThrustControllerNode::TakingoffCallback,this);

  // twist_pub_ = nh.advertise<geometry_msgs::TwistStamped>("cmd_twist",10);
  timer_=nh.createTimer(ros::Duration(0.01),&RollPitchYawrateThrustControllerNode::TimerCallback,this);
  ROS_INFO_STREAM("hellp ");
}
bool RollPitchYawrateThrustControllerNode::TakingoffCallback(
		std_srvs::Trigger::Request &req,
		std_srvs::Trigger::Response &res){
	roll_pitch_yawrate_thrust_controller_.SetTakingoff(true);
	res.message="take off success";
	res.success=true;
	return true;
}
void RollPitchYawrateThrustControllerNode::TimerCallback(const ros::TimerEvent & e){

  if(roll_pitch_yawrate_thrust_controller_.GetTakingoff() && odometry_.position(2)>take_off_height_ ){
	  //first stop take off behavior
	  roll_pitch_yawrate_thrust_controller_.SetTakingoff(false);
	  ROS_INFO("take off finished!");
	  //next send to svo to restart the position calculation.
	  std_srvs::Trigger srv;
	  if(svo_control_client_.call(srv)){
		ROS_INFO("message: %s",srv.response.message.c_str());
	  }else{
		ROS_ERROR("Failed to call service add_two_ints");
	  }
  }else{
	// ROS_INFO("%lf", odometry_.position(2));
  }
  // twist_pub_.publish(roll_pitch_yawrate_thrust_controller_.getTwist());
}

void RollPitchYawrateThrustControllerNode::MultiDofJointTrajectoryCallback(
    const trajectory_msgs::MultiDOFJointTrajectoryConstPtr& msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  const size_t n_commands = msg->points.size();

  if(n_commands < 1){
    ROS_WARN_STREAM("Got MultiDOFJointTrajectory message, but message has no points.");
    return;
  }

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromMsg(msg->points.front(), &eigen_reference);
  commands_.push_front(eigen_reference);

  for (size_t i = 1; i < n_commands; ++i) {
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& reference_before = msg->points[i-1];
    const trajectory_msgs::MultiDOFJointTrajectoryPoint& current_reference = msg->points[i];

    mav_msgs::eigenTrajectoryPointFromMsg(current_reference, &eigen_reference);

    commands_.push_back(eigen_reference);
    command_waiting_times_.push_back(current_reference.time_from_start - reference_before.time_from_start);
  }

  // We can trigger the first command immediately.
  roll_pitch_yawrate_thrust_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if (n_commands > 1) {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}
void RollPitchYawrateThrustControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {

  if(commands_.empty()){
    ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
  roll_pitch_yawrate_thrust_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
  command_timer_.stop();
  if(!command_waiting_times_.empty()){
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

RollPitchYawrateThrustControllerNode::~RollPitchYawrateThrustControllerNode() { }

void RollPitchYawrateThrustControllerNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // Read parameters from rosparam.
  GetRosParameter(pnh, "attitude_gain/x",
                  roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.x(),
                  &roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(pnh, "attitude_gain/y",
                  roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.y(),
                  &roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(pnh, "attitude_gain/z",
                  roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.z(),
                  &roll_pitch_yawrate_thrust_controller_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(pnh, "angular_rate_gain/x",
                  roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(pnh, "angular_rate_gain/y",
                  roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(pnh, "angular_rate_gain/z",
                  roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &roll_pitch_yawrate_thrust_controller_.controller_parameters_.angular_rate_gain_.z());
  GetVehicleParameters(pnh, &roll_pitch_yawrate_thrust_controller_.vehicle_parameters_);
  roll_pitch_yawrate_thrust_controller_.InitializeParameters();
}
void RollPitchYawrateThrustControllerNode::Publish() {
}

void RollPitchYawrateThrustControllerNode::RollPitchYawrateThrustCallback(
    const mav_msgs::RollPitchYawrateThrustConstPtr& roll_pitch_yawrate_thrust_reference_msg) {
  mav_msgs::EigenRollPitchYawrateThrust roll_pitch_yawrate_thrust;
  mav_msgs::eigenRollPitchYawrateThrustFromMsg(*roll_pitch_yawrate_thrust_reference_msg, &roll_pitch_yawrate_thrust);
  roll_pitch_yawrate_thrust_controller_.SetRollPitchYawrateThrust(roll_pitch_yawrate_thrust);
}


void RollPitchYawrateThrustControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("RollPitchYawrateThrustController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  odometry_=odometry;
  roll_pitch_yawrate_thrust_controller_.SetOdometry(odometry);

  Eigen::VectorXd ref_rotor_velocities;
  roll_pitch_yawrate_thrust_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = odometry_msg->header.stamp;

  motor_velocity_reference_pub_.publish(actuator_msg);
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "roll_pitch_yawrate_thrust_controller_node");

  rotors_control::RollPitchYawrateThrustControllerNode roll_pitch_yawrate_thrust_controller_node;

  ros::spin();

  return 0;
}
