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
      start_formation_control_(false),
      f1_("/home/zbf/hello.txt",std::ios::out) {
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
  // f1_ = std::ofstream("/home/zbf/hello.txt",std::ios::out);
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
    // s1_int_(index) = maxmin(s1_int_(index), 0.1,-0.1);
    s1 = x2(index) + s1_int_(index);
    // s2_int_(index) = s2_int_(index) + sign(s1)*dt;
    // s2 = -k2 * s2_int_(index) + Delta(index);
    u_t3 = -k1_ * pow(fabs(s1),0.5) * sign(s1);
    u_t4_int_(index) = u_t4_int_(index) + sign(s1)*dt;
    // u_t4_int_(index) = maxmin(u_t4_int_(index), 0.1,-0.1);
    u_t4 = -k2_* u_t4_int_(index);
    u(index) = u_t1 + u_t2 + u_t3+ u_t4;
  }
  static Eigen::Vector3d acc;
  acc =  
      ( position_error.cwiseProduct(controller_parameters_.position_gain_)
      + velocity_error.cwiseProduct(controller_parameters_.velocity_gain_)
      ) / vehicle_parameters_.mass_
      - vehicle_parameters_.gravity_ * e_3 - command_trajectory_.acceleration_W;
  // std::cout <<"here is always running" <<std::endl;
  acc(0) = -u(0);
            // position_error(0) / vehicle_parameters_.mass_ *  controller_parameters_.position_gain_(0)+ 
            // velocity_error(0) / vehicle_parameters_.mass_ * controller_parameters_.velocity_gain_(0);
  acc(1) = -u(1);
            // position_error(1) / vehicle_parameters_.mass_ *  controller_parameters_.position_gain_(1)+ 
            // velocity_error(1) / vehicle_parameters_.mass_ * controller_parameters_.velocity_gain_(1);
  acc(2) = -vehicle_parameters_.gravity_ - u(2);
            // position_error(2) / vehicle_parameters_.mass_ * controller_parameters_.position_gain_(2) + 
            // velocity_error(2) / vehicle_parameters_.mass_ * controller_parameters_.velocity_gain_(2);
  // double temp;
  // temp = u(0);
  // u(0) = -u(1);
  // u(1) = -temp;
  // u(2) = -u(2);
  // acc(2) = -u(2);
  *acceleration =acc;
  // *acceleration = -u;//- vehicle_parameters_.gravity_ * e_3 - u;
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
  double yaw = command_trajectory_.getYaw();

  // Get the desired rotation matrix.
  Eigen::Vector3d b1_des;
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

  Eigen::Vector3d Theta;
  Eigen::Vector3d Theta_d;
  Theta_d(2) = yaw;
  Theta_d(0) = asin(sin(-(-sin(yaw)*acceleration(0) - cos(yaw)*acceleration(1)) / vehicle_parameters_.gravity_));
  Theta_d(1) = asin(sin(-(cos(yaw)*acceleration(0) + sin(yaw)*acceleration(1)) / vehicle_parameters_.gravity_));
  
  Eigen::Vector3d euler = R.eulerAngles(2,1,0).transpose();
  Theta(0) = asin(sin(euler(0)));
  Theta(1) = asin(sin(euler(1)));
  // Theta(2) = asin(sin(euler(2)));
  Theta(2) =2*asin(sin(euler(2)*0.5));
  // Eigen::Matrix3d R_des;

  // R_des = 
  //      Eigen::AngleAxisd(Theta_d(2), Eigen::Vector3d::UnitZ())  // yaw
  //    * Eigen::AngleAxisd(Theta_d(0), Eigen::Vector3d::UnitX())  // pitch
  //    * Eigen::AngleAxisd(Theta_d(1), Eigen::Vector3d::UnitY());  // roll

  f1_ <<Theta_d(0) <<"," << Theta_d(1)<<"," << Theta_d(2)<<","
      <<Theta(0) <<"," << Theta(1)<<"," << Theta(2)<< "," <<
      acceleration(0) <<"," << acceleration(1)<<"," << acceleration(2)<< std::endl;
  
  // angle_error_ = Theta - Theta_d;
  // Angle error according to lee et al.
  Eigen::Matrix3d angle_error_matrix = 0.5 * (R_des.transpose() * R - R.transpose() * R_des);
  angle_ = rotationMatrix2Eular(R_des); 
  vectorFromSkewMatrix(angle_error_matrix, &angle_error_);

  // TODO(burrimi) include angular rate references at some point.
  Eigen::Vector3d angular_rate_des(Eigen::Vector3d::Zero());
  angular_rate_des[2] = command_trajectory_.getYawRate();
  Eigen::Vector3d angle_gain;
  Eigen::Vector3d attitude_gain;
  angle_gain << 0.37,0.37,0.0135;
  attitude_gain << 0.37*143,0.37*143,0.0135*83.3;
  angular_rate_des = - angle_error_.cwiseProduct(angle_gain);

  Eigen::Vector3d angular_acc_des(Eigen::Vector3d::Zero());
  angular_acc_des[2] = command_trajectory_.getYawAcc();

  // Eigen::Vector3d angular_rate_error = odometry_.angular_velocity - angular_rate_des;
  Eigen::Vector3d angular_rate_error = odometry_.angular_velocity
     - R_des.transpose() * R * angular_rate_des;
  Eigen::Matrix3d J = vehicle_parameters_.inertia_;
  Eigen::Matrix3d skew_angular;
  Eigen::Vector3d& v=odometry_.angular_velocity;
  cskewMatrixFromVector(odometry_.angular_velocity,&skew_angular);
  // 这里面要加两项
  // 一项是角度的偏差
  // 一项是角速度的偏差
  static Eigen::Vector3d pre_angular_rate_error;
  double dt = 0.01;
  Eigen::Vector3d angular_rate_error_d;
  angular_rate_error_d = -(angular_rate_error - pre_angular_rate_error)/dt;
  Eigen::Vector3d angular_rate_error_d_gain;
  angular_rate_error_d_gain << 0.1*143,0.1*143,0.025*83.3;
  *angular_acceleration = 
                          - angle_error_.cwiseProduct(attitude_gain)
                          // - angle_error_.cwiseProduct(normalized_attitude_gain_)
                          // - angular_rate_error_d.cwiseProduct(angular_rate_error_d_gain)
                          // - angular_rate_error.cwiseProduct(normalized_angular_rate_gain_)
                          - angular_rate_error.cwiseProduct(angular_rate_error_d_gain)
                          + odometry_.angular_velocity.cross(J * odometry_.angular_velocity)
                          - J*(skew_angular*R.transpose()*R_des*angular_rate_des
                          - R.transpose()*R_des*angular_acc_des) // we don't need the inertia matrix here
                          + Eigen::Vector3d(0,0,0);
  pre_angular_rate_error = angular_rate_error;
}

double SlidingModeController::sign(double input){
  if(input>0)return 1.0;
  if(input<0)return -1.0;
  return 0.0;
}
double SlidingModeController::maxmin(double value, double max_th, double min_th){
  if(value>max_th)return max_th;
  if(value<min_th)return min_th;
  return value;
}
}
