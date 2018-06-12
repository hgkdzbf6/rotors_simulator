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

#include "sliding_mode_node.h"

#include "rotors_control/parameters_ros.h"

namespace rotors_control {

SlidingModeNode::SlidingModeNode(): start_formation_control_(false) {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  InitializeParams();

  cmd_pose_sub_ = nh.subscribe(
      mav_msgs::default_topics::COMMAND_POSE, 1,
      &SlidingModeNode::CommandPoseCallback, this);

  cmd_multi_dof_joint_trajectory_sub_ = nh.subscribe(
      mav_msgs::default_topics::COMMAND_TRAJECTORY, 1,
      &SlidingModeNode::MultiDofJointTrajectoryCallback, this);

  odometry_sub_ = nh.subscribe(mav_msgs::default_topics::ODOMETRY, 1,
                               &SlidingModeNode::OdometryCallback, this);

  // 这里把leader的位置映射成odometry, 除了还要引入什么
  // 在leader坐标系下的follower位置
  pnh.param<std::string>("relative_pose",relative_pose_str_,"/hummingbird0/relative_pose01");
  leader_position_sub_ = nh.subscribe(relative_pose_str_,1,&SlidingModeNode::LeaderPositionCallback,this );
  // 在leader坐标系下的follower的desired位置
  leader_desired_pose_sub_ =nh.subscribe("leader_desired_pose",1,&SlidingModeNode::LeaderDesiredPositionCallback,this);
  
  // 起飞服务
  taking_off_server_ = nh.advertiseService("taking_off",
		  &SlidingModeNode::TakingoffCallback,this);

  // svo控制
  svo_control_client_ = nh.serviceClient<std_srvs::Trigger>("svo_control");

  motor_velocity_reference_pub_ = nh.advertise<mav_msgs::Actuators>(
      mav_msgs::default_topics::COMMAND_ACTUATORS, 1);

  command_timer_ = nh.createTimer(ros::Duration(0), &SlidingModeNode::TimedCommandCallback, this,
                                  true, false);

  twist_pub_ = nh.advertise<geometry_msgs::TwistStamped>("cmd_twist",10);
  timer_=nh.createTimer(ros::Duration(0.01),&SlidingModeNode::TimerCallback,this);
}

SlidingModeNode::~SlidingModeNode() { }

bool SlidingModeNode::TakingoffCallback(
		std_srvs::Trigger::Request &req,
		std_srvs::Trigger::Response &res){
	sliding_mode_controller_.SetTakingoff(true);
	res.message="take off success";
	res.success=true;
	return true;
}

void SlidingModeNode::InitializeParams() {
  ros::NodeHandle pnh("~");

  // Read parameters from rosparam.
  GetRosParameter(pnh, "attitude_gain/x",
                  sliding_mode_controller_.controller_parameters_.attitude_gain_.x(),
                  &sliding_mode_controller_.controller_parameters_.attitude_gain_.x());
  GetRosParameter(pnh, "attitude_gain/y",
                  sliding_mode_controller_.controller_parameters_.attitude_gain_.y(),
                  &sliding_mode_controller_.controller_parameters_.attitude_gain_.y());
  GetRosParameter(pnh, "attitude_gain/z",
                  sliding_mode_controller_.controller_parameters_.attitude_gain_.z(),
                  &sliding_mode_controller_.controller_parameters_.attitude_gain_.z());
  GetRosParameter(pnh, "angular_rate_gain/x",
                  sliding_mode_controller_.controller_parameters_.angular_rate_gain_.x(),
                  &sliding_mode_controller_.controller_parameters_.angular_rate_gain_.x());
  GetRosParameter(pnh, "angular_rate_gain/y",
                  sliding_mode_controller_.controller_parameters_.angular_rate_gain_.y(),
                  &sliding_mode_controller_.controller_parameters_.angular_rate_gain_.y());
  GetRosParameter(pnh, "angular_rate_gain/z",
                  sliding_mode_controller_.controller_parameters_.angular_rate_gain_.z(),
                  &sliding_mode_controller_.controller_parameters_.angular_rate_gain_.z());

  pnh.param<double >("take_off_height", take_off_height_, 2);

  GetVehicleParameters(pnh, &sliding_mode_controller_.vehicle_parameters_);
  sliding_mode_controller_.InitializeParameters();
}
void SlidingModeNode::Publish() {
}

void SlidingModeNode::CommandPoseCallback(
    const geometry_msgs::PoseStampedConstPtr& pose_msg) {
  // Clear all pending commands.
  command_timer_.stop();
  commands_.clear();
  command_waiting_times_.clear();

  mav_msgs::EigenTrajectoryPoint eigen_reference;
  mav_msgs::eigenTrajectoryPointFromPoseMsg(*pose_msg, &eigen_reference);
  commands_.push_front(eigen_reference);

  sliding_mode_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
}

void SlidingModeNode::MultiDofJointTrajectoryCallback(
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
  sliding_mode_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();

  if (n_commands > 1) {
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}
void SlidingModeNode::TimerCallback(const ros::TimerEvent & e){

  if(sliding_mode_controller_.GetTakingoff() && odometry_.position(2)>take_off_height_ ){
	  //first stop take off behavior
	  sliding_mode_controller_.SetTakingoff(false);
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

  Eigen::VectorXd ref_rotor_velocities;
  sliding_mode_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  // Todo(ffurrer): Do this in the conversions header.
  mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  actuator_msg->angular_velocities.clear();
  for (int i = 0; i < ref_rotor_velocities.size(); i++)
    actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  actuator_msg->header.stamp = last_odometry_msg_stamp_;
  Eigen::Matrix3d R  =  (odometry_.orientation).toRotationMatrix();
  Eigen::Quaterniond q = odometry_.orientation;
  double roll,pitch,yaw;
  Eigen::Vector3d euler = R.eulerAngles(2,1,0).transpose();
  roll = asin(sin(euler(0)));
  pitch = asin(sin(euler(1)));
  // yaw =2*asin(sin(euler(2)*0.5));
  yaw =asin(sin(euler(2)));
  // ROS_INFO_STREAM("roll: " << roll <<" pitch: " << pitch <<" yaw: " << yaw);
  // ROS_INFO_STREAM(odometry_.position.transpose());
  // ROS_INFO_STREAM("w: " << q.w() << " x: "<<q.x()<<" y: "<<q.y()<<" z: "<<q.z());
  motor_velocity_reference_pub_.publish(actuator_msg);

  twist_pub_.publish(sliding_mode_controller_.getTwist());
}

void SlidingModeNode::TimedCommandCallback(const ros::TimerEvent& e) {

  if(commands_.empty()){
    ROS_WARN("Commands empty, this should not happen here");
    return;
  }

  const mav_msgs::EigenTrajectoryPoint eigen_reference = commands_.front();
  sliding_mode_controller_.SetTrajectoryPoint(commands_.front());
  commands_.pop_front();
  command_timer_.stop();
  if(!command_waiting_times_.empty()){
    command_timer_.setPeriod(command_waiting_times_.front());
    command_waiting_times_.pop_front();
    command_timer_.start();
  }
}

void SlidingModeNode::LeaderDesiredPositionCallback(const geometry_msgs::PoseStampedConstPtr& msg) {

  ROS_INFO_ONCE("Leader Desired Position Controller got first leader position message.");
  sliding_mode_controller_.start_formation_control_=true;
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
  sliding_mode_controller_.SetLeaderDesiredOdometry(odometry);
}

void SlidingModeNode::LeaderPositionCallback(const geometry_msgs::PoseStampedConstPtr& msg) {

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
  sliding_mode_controller_.SetLeaderOdometry(odometry);
}

void SlidingModeNode::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg) {

  ROS_INFO_ONCE("SlidingModeController got first odometry message.");

  EigenOdometry odometry;
  eigenOdometryFromMsg(odometry_msg, &odometry);
  sliding_mode_controller_.SetOdometry(odometry);
  last_odometry_msg_stamp_ =odometry_msg->header.stamp;
  odometry_=odometry;

  // Eigen::VectorXd ref_rotor_velocities;
  // sliding_mode_controller_.CalculateRotorVelocities(&ref_rotor_velocities);

  // // Todo(ffurrer): Do this in the conversions header.
  // mav_msgs::ActuatorsPtr actuator_msg(new mav_msgs::Actuators);

  // actuator_msg->angular_velocities.clear();
  // for (int i = 0; i < ref_rotor_velocities.size(); i++)
  //   actuator_msg->angular_velocities.push_back(ref_rotor_velocities[i]);
  // actuator_msg->header.stamp = odometry_msg->header.stamp;
  // Eigen::Matrix3d R  =  (odometry_.orientation).toRotationMatrix();
  // Eigen::Quaterniond q = odometry_.orientation;
  // ROS_INFO_STREAM(R.eulerAngles(2,1,0).transpose());
  // ROS_INFO_STREAM("w: "<<q.w()<<" x: "<<q.x()<<" y: "<<q.y()<<" z: "<<q.z());
  // motor_velocity_reference_pub_.publish(actuator_msg);
}

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sliding_mode_controller_node");

  rotors_control::SlidingModeNode sliding_mode_controller_node;
  ros::spin();

  return 0;
}
