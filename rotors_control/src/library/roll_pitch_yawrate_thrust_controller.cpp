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

#include "rotors_control/roll_pitch_yawrate_thrust_controller.h"

namespace rotors_control {

RollPitchYawrateThrustController::RollPitchYawrateThrustController()
    : initialized_params_(false),
      controller_active_(false) {
  InitializeParameters();
}

RollPitchYawrateThrustController::~RollPitchYawrateThrustController() {}
void RollPitchYawrateThrustController::SetEnable(bool val){
	controller_active_=val;
  // ROS_INFO("enabled..");
}
bool RollPitchYawrateThrustController::GetTakingoff(){
	return taking_off_;
}
void RollPitchYawrateThrustController::SetTakingoff(bool val){
	taking_off_=val;
	SetEnable(true);
  // ROS_INFO("take off set.");
}
void RollPitchYawrateThrustController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
}
void RollPitchYawrateThrustController::InitializeParameters() {
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
  
  s1_int_.setZero();
  s2_int_.setZero();
  u_t4_int_.setZero();

  k1_ = controller_parameters_.k1;
  k2_ = controller_parameters_.k2;
  lambda1_ = controller_parameters_.lambda1;
  lambda2_ = controller_parameters_.lambda2;
  rho_1_ = controller_parameters_.rho_1;
  rho_2_ = controller_parameters_.rho_2;
  
  initialized_params_ = true;
}

void RollPitchYawrateThrustController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities)  {
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
    acceleration = Eigen::Vector3d(0,0,-15.9);
  }else{
     ComputeDesiredAcceleration(&acceleration);
    //acceleration = Eigen::Vector3d(0,0,-15.9);
  }

  ROS_INFO_STREAM("accel : ");
  Eigen::Vector3d angular_acceleration;
  ComputeDesiredAngularAcc(acceleration, &angular_acceleration);
  double thrust = -vehicle_parameters_.mass_ * acceleration.dot(odometry_.orientation.toRotationMatrix().col(2));

  if(taking_off_){
	  thrust=9;
  }
  
  std::cout << thrust << std::endl;
  Eigen::Vector4d angular_acceleration_thrust;
  angular_acceleration_thrust.block<3, 1>(0, 0) = angular_acceleration;
  angular_acceleration_thrust(3) = thrust;//roll_pitch_yawrate_thrust_.thrust.z();

  *rotor_velocities = angular_acc_to_rotor_velocities_ * angular_acceleration_thrust;
  *rotor_velocities = rotor_velocities->cwiseMax(Eigen::VectorXd::Zero(rotor_velocities->rows()));
  *rotor_velocities = rotor_velocities->cwiseSqrt();
}

void RollPitchYawrateThrustController::ComputeDesiredAcceleration(Eigen::Vector3d* acceleration)  {
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
  
  Eigen::Vector3d u;
  Eigen::Vector3d & x1 = position_error;
  Eigen::Vector3d & x2 = velocity_error;
  
  std::cout <<"x1 : "<< x1.transpose() << std::endl;
  std::cout <<"x2 : "<< x2.transpose() << std::endl;
  double u_t1,u_t2,u_t3,u_t4;
  double s1;
  double dt = 0.1;
  // 开始滑模控制
  for(int index = 0;index<3;index++){
    u_t1 = -lambda1_ * pow(fabs(x1(index)),rho_1_) * sign(x1(index));
    u_t2 = -lambda2_ * pow(fabs(x2(index)),rho_2_) * sign(x2(index));
    s1_int_(index) = s1_int_(index) + (-u_t1-u_t2)*dt;
    s1 = x2(index) + s1_int_(index);
    // s2_int_(index) = s2_int_(index) + sign(s1)*dt;
    // s2 = -k2 * s2_int_(index) + Delta(index);
    u_t3 = -k1_ * pow(fabs(s1),0.5) * sign(s1);
    u_t4_int_(index) = u_t4_int_(index) + sign(s1)*dt;
    u_t4 = -k2_* u_t4_int_(index);
    u(index) = u_t1 + u_t2 + u_t3+ u_t4;
  }

  *acceleration = u;
  u(2) = -u(2);
      //  u / vehicle_parameters_.mass_
      //  - vehicle_parameters_.gravity_ * e_3 - command_trajectory_.acceleration_W;
  std::cout <<"accel : "<< (*acceleration).transpose() << std::endl;
}
void RollPitchYawrateThrustController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}

void RollPitchYawrateThrustController::SetRollPitchYawrateThrust(
    const mav_msgs::EigenRollPitchYawrateThrust& roll_pitch_yawrate_thrust) {
  roll_pitch_yawrate_thrust_ = roll_pitch_yawrate_thrust;
  controller_active_ = true;
}

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void RollPitchYawrateThrustController::ComputeDesiredAngularAcc(Eigen::Vector3d acceleration, Eigen::Vector3d* angular_acceleration) const {
  assert(angular_acceleration);

  Eigen::Matrix3d R = odometry_.orientation.toRotationMatrix();
  double yaw = atan2(R(1, 0), R(0, 0));

  // Get the desired rotation matrix.
  Eigen::Matrix3d R_des;
  std::cout  <<  acceleration.transpose() <<std::endl;
  std::cout  <<  yaw <<std::endl;
  R_des = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())  // yaw
        * Eigen::AngleAxisd(asin(acceleration(0) / (acceleration(2) + vehicle_parameters_.gravity_)), Eigen::Vector3d::UnitX())  // roll
        * Eigen::AngleAxisd(asin(acceleration(1) / (acceleration(2) + vehicle_parameters_.gravity_) ), Eigen::Vector3d::UnitY());  // pitch

  // Angle error according to lee et al.
  Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
  Eigen::Vector3d angle_error;
  vectorFromSkewMatrix(angle_error_matrix, &angle_error);

  // TODO(burrimi) include angular rate references at some point.
  Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
  angular_rate_des[2] = roll_pitch_yawrate_thrust_.yaw_rate;

  Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - R_des.transpose() * R * angular_rate_des;

  *angular_acceleration = -1 * angle_error.cwiseProduct(normalized_attitude_gain_)
                           - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
                           + odometry_.angular_velocity.cross(odometry_.angular_velocity); // we don't need the inertia matrix here
}
double RollPitchYawrateThrustController::sign(double input){
  if(input>0)return 1.0;
  if(input<0)return -1.0;
  return 0.0;
}
}
