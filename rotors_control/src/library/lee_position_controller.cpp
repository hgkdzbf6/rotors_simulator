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

#include "rotors_control/lee_position_controller.h"

namespace rotors_control {

LeePositionController::LeePositionController()
    : initialized_params_(false),
      controller_active_(false),
      start_formation_control_(false) {
  InitializeParameters();
}

LeePositionController::~LeePositionController() {}

void LeePositionController::InitializeParameters() {

  calculateAllocationMatrix(vehicle_parameters_.rotor_configuration_, &(controller_parameters_.allocation_matrix_));
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_attitude_gain_ = controller_parameters_.attitude_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();
  // To make the tuning independent of the inertia matrix we divide here.
  normalized_angular_rate_gain_ = controller_parameters_.angular_rate_gain_.transpose()
      * vehicle_parameters_.inertia_.inverse();

  Eigen::Matrix4d I;
  I.setZero();
  I.block<3, 3>(0, 0) = vehicle_parameters_.inertia_;
  I(3, 3) = 1;
  angular_acc_to_rotor_velocities_.resize(vehicle_parameters_.rotor_configuration_.rotors.size(), 4);
  // Calculate the pseude-inverse A^{ \dagger} and then multiply by the inertia matrix I.
  // A^{ \dagger} = A^T*(A*A^T)^{-1}
  angular_acc_to_rotor_velocities_ = controller_parameters_.allocation_matrix_.transpose()
      * (controller_parameters_.allocation_matrix_
      * controller_parameters_.allocation_matrix_.transpose()).inverse() * I;
  initialized_params_ = true;
}

void LeePositionController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities)  {
  assert(rotor_velocities);
  assert(initialized_params_);

  rotor_velocities->resize(vehicle_parameters_.rotor_configuration_.rotors.size());
  // Return 0 velocities on all rotors, until the first command is received.
  if (!controller_active_) {
    *rotor_velocities = Eigen::VectorXd::Zero(rotor_velocities->rows());
    return;
  }

  Eigen::Vector3d acceleration;

  if(taking_off_){
	  acceleration=Eigen::Vector3d(0,0,-15.9);
  }else{
    ComputeDesiredAcceleration(&acceleration);
  }

  Eigen::Vector3d angular_acceleration;
  ComputeDesiredAngularAcc(acceleration, &angular_acceleration);

  // Project thrust onto body z axis.
  double thrust = -vehicle_parameters_.mass_ * acceleration.dot(odometry_.orientation.toRotationMatrix().col(2));

  if(taking_off_){
	  thrust=9;
  }
  
  // twist_.linear.z = thrust;
  twist_.twist.linear.z = odometry_.position(2);
  twist_cmd_ = twist_; 
  Eigen::Vector4d angular_acceleration_thrust;
  angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
  angular_acceleration_thrust(3) = thrust;

  *rotor_velocities = angular_acc_to_rotor_velocities_ * angular_acceleration_thrust;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
}

void LeePositionController::SetEnable(bool val){
	controller_active_=val;
  // ROS_INFO("enabled..");
}
bool LeePositionController::GetTakingoff(){
	return taking_off_;
}
void LeePositionController::SetTakingoff(bool val){
	taking_off_=val;
	SetEnable(true);
  // ROS_INFO("take off set.");
}

void LeePositionController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}

void LeePositionController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
}

void LeePositionController::SetLeaderOdometry(const EigenOdometry& odometry) {
  leader_odometry_ = odometry;
  new_data_approach_=true;
}

void LeePositionController::SetLeaderDesiredOdometry(const EigenOdometry& odometry) {
  leader_desired_odometry_ = odometry;
}

void LeePositionController::ComputeDesiredAcceleration(Eigen::Vector3d* acceleration)  {
  assert(acceleration);

  Eigen::Vector3d e_3(Eigen::Vector3d::UnitZ());
  // 需要哪些信息?
  // 位置信息,期望位置信息
  // 速度信息,期望速度信息
  // leader的旋转矩阵
  // float theta = atan2(leader_odometry_.position(1),leader_odometry_.position(0));
  Eigen::Vector3d leader_position_error;
  leader_position_error = command_trajectory_.position_W;

  // static Eigen::Vector3d position_error_accumulate;
  Eigen::Vector3d position_error;
  // position_error = odometry_.position - command_trajectory_.position_W - position_error_accumulate;
  // position_error = odometry_.position - command_trajectory_.position_W;
  position_error = odometry_.position - leader_position_error;

  // Transform velocity to world frame.
  const Eigen::Matrix3d R_W_I = odometry_.orientation.toRotationMatrix();
  Eigen::Vector3d velocity_W =  R_W_I * odometry_.velocity;
  Eigen::Vector3d velocity_error;
  velocity_error = velocity_W - command_trajectory_.velocity_W;
  
  *acceleration = 
      ( position_error.cwiseProduct(controller_parameters_.position_gain_)
      + velocity_error.cwiseProduct(controller_parameters_.velocity_gain_)
      ) / vehicle_parameters_.mass_
      - vehicle_parameters_.gravity_ * e_3 - command_trajectory_.acceleration_W;

}

Eigen::Vector3d LeePositionController::rotationMatrix2Eular(Eigen::Matrix3d R){
  Eigen::Vector3d eular;
  // 这个顺序究竟是怎么样的?
  eular = R.eulerAngles(0,1,2);
  return eular;
}

geometry_msgs::TwistStamped LeePositionController::getTwist(){
  return twist_cmd_;
}

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void LeePositionController::ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                                     Eigen::Vector3d* angular_acceleration) {
  assert(angular_acceleration);

  Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();

  Eigen::Matrix3d R_l = leader_odometry_.orientation.toRotationMatrix();
  Eigen::Matrix3d R_l_d = leader_desired_odometry_.orientation.toRotationMatrix();

  // Get the desired rotation matrix.
  Eigen::Vector3d b1_des;
  double yaw = command_trajectory_.getYaw();
  b1_des << cos(yaw), sin(yaw), 0;

  Eigen::Vector3d b3_des;
  b3_des = -acceleration / acceleration.norm();

  Eigen::Vector3d b2_des;
  b2_des = b3_des.cross(b1_des);
  b2_des.normalize();

  Eigen::Matrix3d R_des;
  R_des.col(0) = b2_des.cross(b3_des);
  R_des.col(1) = b2_des;
  R_des.col(2) = b3_des;

  // Angle error according to lee et al.
  Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
  angle_ = rotationMatrix2Eular(R_des);
  twist_.twist.linear.x = angle_(0);
  twist_.twist.linear.y = angle_(1);
  twist_.twist.angular.z = angle_error_(2); 
  vectorFromSkewMatrix(angle_error_matrix, &angle_error_);

  // TODO(burrimi) include angular rate references at some point.
  Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
  angular_rate_des[2] = command_trajectory_.getYawRate();

  Eigen::Vector3d angular_acc_des(Eigen::Vector3d::Zero());
  angular_acc_des[2] = command_trajectory_.getYawAcc();

  Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - R_des.transpose() * R * angular_rate_des;

  Eigen::Matrix3d J = vehicle_parameters_.inertia_;
  Eigen::Matrix3d skew_angular;
  //Eigen::Vector3d& v=odometry_.angular_velocity;
  cskewMatrixFromVector(odometry_.angular_velocity,&skew_angular);
  // 这里面要加两项
  // 一项是角度的偏差
  // 一项是角速度的偏差
  *angular_acceleration = -1 * angle_error_.cwiseProduct(normalized_attitude_gain_)
                          - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
                          + odometry_.angular_velocity.cross(J * odometry_.angular_velocity)
                          - J*(skew_angular*R.transpose()*R_des*angular_rate_des
                          - R.transpose()*R_des*angular_acc_des); // we don't need the inertia matrix here

}
}
