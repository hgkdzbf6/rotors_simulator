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

#include "rotors_control/sliding_mode_controller.h"

namespace rotors_control {

SlidingModeController::SlidingModeController()
    : initialized_params_(false),
      controller_active_(false),
      start_formation_control_(false) {
  InitializeParameters();
}

SlidingModeController::~SlidingModeController() {}

void SlidingModeController::InitializeParameters() {

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

void SlidingModeController::CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities)  {
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

void SlidingModeController::SetEnable(bool val){
	controller_active_=val;
  // ROS_INFO("enabled..");
}
bool SlidingModeController::GetTakingoff(){
	return taking_off_;
}
void SlidingModeController::SetTakingoff(bool val){
	taking_off_=val;
	SetEnable(true);
  // ROS_INFO("take off set.");
}

void SlidingModeController::SetOdometry(const EigenOdometry& odometry) {
  odometry_ = odometry;
}

void SlidingModeController::SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory) {
  command_trajectory_ = command_trajectory;
}

void SlidingModeController::SetLeaderOdometry(const EigenOdometry& odometry) {
  leader_odometry_ = odometry;
  new_data_approach_=true;
}

void SlidingModeController::SetLeaderDesiredOdometry(const EigenOdometry& odometry) {
  leader_desired_odometry_ = odometry;
}

void SlidingModeController::ComputeDesiredAcceleration(Eigen::Vector3d* acceleration)  {
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
  double u_t1,u_t2,u_t3,u_t4;
  double s1;
  double dt = 0.01;
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
  double temp;
  temp = u(0);
  u(0) = -u(1);
  u(1) = -temp;
  u(2) = -u(2);
  // *acceleration = 
  //     ( position_error.cwiseProduct(controller_parameters_.position_gain_)
  //     + velocity_error.cwiseProduct(controller_parameters_.velocity_gain_)
  //     ) / vehicle_parameters_.mass_
  //     - vehicle_parameters_.gravity_ * e_3 - command_trajectory_.acceleration_W;
  *acceleration = u;
}

Eigen::Vector3d SlidingModeController::rotationMatrix2Eular(Eigen::Matrix3d R){
  Eigen::Vector3d eular;
  // 这个顺序究竟是怎么样的?
  eular = R.eulerAngles(2,1,2);
  return eular;
}

geometry_msgs::TwistStamped SlidingModeController::getTwist(){
  return twist_cmd_;
}

// Implementation from the T. Lee et al. paper
// Control of complex maneuvers for a quadrotor UAV using geometric methods on SE(3)
void SlidingModeController::ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
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
double SlidingModeController::sign(double input){
  if(input>0)return 1.0;
  if(input<0)return -1.0;
  return 0.0;
}
}
