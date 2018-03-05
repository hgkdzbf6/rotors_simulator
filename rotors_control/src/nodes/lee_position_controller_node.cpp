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

#include <ros/ros.h>
#include <mav_msgs/default_topics.h>

#include "lee_position_controller_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

LeePositionControllerNode::LeePositionControllerNode(): start_formation_control_(false) {
  ros::NodeHandle nh;
  InitializeParams();

  cmd_pose_sub_ = nh.subscribe(
      mav_msgs::default_topics::COMMAND_POSE, 1,
      &LeePositionControllerNode::CommandPoseCallback, this);

  cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &LeePositionControllerNode::MultiDofJointTrajectoryCallback, this);

  odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                               &LeePositionControllerNode::OdometryCallback, this);

  // 这里把leader的位置映射成odometry, 除了还要引入什么
  // 在leader坐标系下的follower位置
  leader_position_sub_ = nh.subscribe("/hummingbird0/relative_position01",1,&LeePositionControllerNode::LeaderPositionCallback,this );
  // 在leader坐标系下的follower的desired位置
  leader_desired_position_sub_ =nh.subscribe("leader_desired_position",1,&LeePositionControllerNode::LeaderDesiredPositionCallback,this);
  
  // 起飞服务
  taking_off_server_ = nh.advertiseService("taking_off",
		  &LeePositionControllerNode::TakingoffCallback,this);

  // svo控制
  svo_control_client_ = nh.serviceClient<std_srvs::Trigger>("svo_control");

  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  command_timer_ = nh.createTimer(ros::Duration(0), &LeePositionControllerNode::TimedCommandCallback, this,
                                  true, false);

  timer_=nh.createTimer(ros::Duration(0.01),&LeePositionControllerNode::TimerCallback,this);
}

LeePositionControllerNode::~LeePositionControllerNode() { }

bool LeePositionControllerNode::TakingoffCallback(
		std_srvs::Trigger::Request &req,
		std_srvs::Trigger::Response &res){
	lee_position_controller_.SetTakingoff(true);
	res.message="take off success";
	res.success=true;
	return true;
}

void LeePositionControllerNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // Read parameters from rosparam.
  GetRosParameter(pnh, "position_gain/x",
                  lee_position_controller_.controller_parameters_.position_gain_.x(),
                  &lee_position_controller_.controller_parameters_.position_gain_.x());
  GetRosParameter(pnh, "position_gain/y",
                  lee_position_controller_.controller_parameters_.position_gain_.y(),
                  &lee_position_controller_.controller_parameters_.position_gain_.y());
  GetRosParameter(pnh, "position_gain/z",
                  lee_position_controller_.controller_parameters_.position_gain_.z(),
                  &lee_position_controller_.controller_parameters_.position_gain_.z());
  GetRosParameter(pnh, "velocity_gain/x",
                  lee_position_controller_.controller_parameters_.velocity_gain_.x(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.x());
  GetRosParameter(pnh, "velocity_gain/y",
                  lee_position_controller_.controller_parameters_.velocity_gain_.y(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.y());
  GetRosParameter(pnh, "velocity_gain/z",
                  lee_position_controller_.controller_parameters_.velocity_gain_.z(),
                  &lee_position_controller_.controller_parameters_.velocity_gain_.z());

  GetRosParameter(pnh, "leader_position_gain/x",
                  lee_position_controller_.controller_parameters_.leader_position_gain_.x(),
                  &lee_position_controller_.controller_parameters_.leader_position_gain_.x());
  GetRosParameter(pnh, "leader_position_gain/y",
                  lee_position_controller_.controller_parameters_.leader_position_gain_.y(),
                  &lee_position_controller_.controller_parameters_.leader_position_gain_.y());
  GetRosParameter(pnh, "leader_position_gain/z",
                  lee_position_controller_.controller_parameters_.leader_position_gain_.z(),
                  &lee_position_controller_.controller_parameters_.leader_position_gain_.z());
  GetRosParameter(pnh, "leader_velocity_gain/x",
                  lee_position_controller_.controller_parameters_.leader_velocity_gain_.x(),
                  &lee_position_controller_.controller_parameters_.leader_velocity_gain_.x());
  GetRosParameter(pnh, "leader_velocity_gain/y",
                  lee_position_controller_.controller_parameters_.leader_velocity_gain_.y(),
                  &lee_position_controller_.controller_parameters_.leader_velocity_gain_.y());
  GetRosParameter(pnh, "leader_velocity_gain/z",
                  lee_position_controller_.controller_parameters_.leader_velocity_gain_.z(),
                  &lee_position_controller_.controller_parameters_.leader_velocity_gain_.z());

  GetRosParameter(pnh, "attitude_gain/x",
                  lee_position_controller_.controller_parameters_.attitude_gain_.x(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(pnh, "attitude_gain/y",
                  lee_position_controller_.controller_parameters_.attitude_gain_.y(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(pnh, "attitude_gain/z",
                  lee_position_controller_.controller_parameters_.attitude_gain_.z(),
                  &lee_position_controller_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(pnh, "angular_rate_gain/x",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(pnh, "angular_rate_gain/y",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(pnh, "angular_rate_gain/z",
                  lee_position_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &lee_position_controller_.controller_parameters_.angular_rate_gain_.z());

  pnh.param<double >("take_off_height", take_off_height_, 2);

  GetVehicleParameters(pnh, &lee_position_controller_.vehicle_parameters_);
  lee_position_controller_.InitializeParameters();
}
void LeePositionControllerNode::Publish() {
}

void LeePositionControllerNode::CommandPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
  commands_.push_front(eigen_reference);

  lee_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
}

void LeePositionControllerNode::MultiDofJointTrajectoryCallback(
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
  lee_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if (n_commands > 1) {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}
void LeePositionControllerNode::TimerCallback(const ros::TimerEvent & e){

  if(lee_position_controller_.GetTakingoff() && odometry_.position(2)>take_off_height_ ){
	  //first stop take off behavior
	  lee_position_controller_.SetTakingoff(false);
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
}

void LeePositionControllerNode::TimedCommandCallback(const ros::TimerEvent& e) {

  if(commands_.empty()){
    ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
  lee_position_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
  command_timer_.stop();
  if(!command_waiting_times_.empty()){
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void LeePositionControllerNode::LeaderDesiredPositionCallback(const geometry_msgs::PoseStampedConstPtr& msg) {

  ROS_INFO_ONCE("Leader Desired Position Controller got first leader position message.");
  lee_position_controller_.start_formation_control_=true;
  static EigenOdometry odometry;

  float dt=0.1f;
  odometry.velocity(0)=(msg->pose.position.x-odometry.position(0))/dt;
  odometry.velocity(1)=(msg->pose.position.y-odometry.position(1))/dt;
  odometry.velocity(2)=(msg->pose.position.z-odometry.position(2))/dt;
  odometry.position(0)=msg->pose.position.x;
  odometry.position(1)=msg->pose.position.y;
  odometry.position(2)=msg->pose.position.z;
  odometry.orientation=Eigen::Quaterniond(msg->pose.orientation.w,
    msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z);

  // ROS_INFO_STREAM("desired position: "<<odometry.position);
  lee_position_controller_.SetLeaderDesiredOdometry(odometry);
}

void LeePositionControllerNode::LeaderPositionCallback(const geometry_msgs::PoseStampedConstPtr& msg) {

  ROS_INFO_ONCE("Leader Position Controller got first leader position message.");
  static EigenOdometry odometry;

  float dt=0.1f;
  odometry.velocity(0)=(msg->pose.position.x-odometry.position(0))/dt;
  odometry.velocity(1)=(msg->pose.position.y-odometry.position(1))/dt;
  odometry.velocity(2)=(msg->pose.position.z-odometry.position(2))/dt;
  odometry.position(0)=msg->pose.position.x;
  odometry.position(1)=msg->pose.position.y;
  odometry.position(2)=msg->pose.position.z;
  odometry.orientation=Eigen::Quaterniond(msg->pose.orientation.w,
    msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z);

  // ROS_INFO_STREAM("position:"<<odometry.position);
  lee_position_controller_.SetLeaderOdometry(odometry);
}

void LeePositionControllerNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("LeePositionController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  lee_position_controller_.SetOdometry(odometry);

  odometry_=odometry;

  Eigen::VectorXd ref_rotor_velocities;
  lee_position_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

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
  ros::init(argc, argv, "lee_position_controller_node");

  rotors_control::LeePositionControllerNode lee_position_controller_node;
  ros::spin();

  return 0;
}
