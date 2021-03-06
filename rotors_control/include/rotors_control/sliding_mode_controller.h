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

#ifndef ROTORS_CONTROL_SLIDING_MODE_CONTROLLER_H
#define ROTORS_CONTROL_SLIDING_MODE_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "rotors_control/common.h"
#include "rotors_control/parameters.h"
#include <geometry_msgs/TwistStamped.h>
#include <iostream>
#include <fstream>
namespace rotors_control {

// Default values for the lee position controller and the Asctec Firefly.
static const Eigen::Vector3d kDefaultPositionGain = Eigen::Vector3d(40, 40, 40);
static const Eigen::Vector3d kDefaultVelocityGain = Eigen::Vector3d(1.7, 1.7, 1.7);
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 3, 0.035);
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 0.52, 0.025);

class SlidingModeControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SlidingModeControllerParameters()
      : k1(0.01),k2(0.01),lambda1(0.6),lambda2(4.7),rho(0.5),
      // : k1(0.2),k2(0.2),lambda1(1.6),lambda2(2.3),rho(0.5),
        k21(0),k22(0),lambda21(53),lambda22(14.3),rho2(0.9),
      position_gain_(kDefaultPositionGain/8),velocity_gain_(kDefaultVelocityGain*0.9),
        attitude_gain_(kDefaultAttitudeGain),
        angular_rate_gain_(kDefaultAngularRateGain) {
          // rho_1 = rho / (2-rho);
          // rho_2 = rho;
          rho_1 = 0.81;
          rho_2 = 0.9;
          rho2_1 = 0.99;
          rho2_2 = 0.99;
    calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
  }

  double k1;
  double k2;
  double lambda1;
  double lambda2;
  double rho;
  double rho_1;
  double rho_2;

  double k21;
  double k22;
  double lambda21;
  double lambda22;
  double rho2;
  double rho2_1;
  double rho2_2;
  Eigen::Matrix4Xd allocation_matrix_;
  Eigen::Vector3d position_gain_;
  Eigen::Vector3d velocity_gain_;
  Eigen::Vector3d attitude_gain_;
  Eigen::Vector3d angular_rate_gain_;
  RotorConfiguration rotor_configuration_;
};

class SlidingModeController {
 public:
  bool start_formation_control_;
  
  SlidingModeController();
  ~SlidingModeController();
  void InitializeParameters();
  void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) ;

  void SetOdometry(const EigenOdometry& odometry);
  void SetLeaderOdometry(const EigenOdometry& odometry);
  void SetLeaderDesiredOdometry(const EigenOdometry& odometry);
  void SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory);

  void SetEnable(bool val);
  void SetTakingoff(bool val);
  bool GetTakingoff();
  Eigen::Vector3d rotationMatrix2Eular(Eigen::Matrix3d R);
  geometry_msgs::TwistStamped getTwist();

  SlidingModeControllerParameters controller_parameters_;
  VehicleParameters vehicle_parameters_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  bool initialized_params_;
  bool controller_active_;
  bool taking_off_;

  bool new_data_approach_;
  double sign(double input);
  double maxmin(double value, double max_th, double min_th);

  Eigen::Vector3d s1_int_;
  Eigen::Vector3d s2_int_;
  Eigen::Vector3d u_t4_int_;

  double k1_;
  double k2_;
  double lambda1_;
  double lambda2_;
  double rho_1_;
  double rho_2_;

  Eigen::VectorXd rotor_velocities_;
  Eigen::Vector3d s21_int_;
  Eigen::Vector3d s22_int_;
  Eigen::Vector3d u2_t4_int_;
  double k21_;
  double k22_;
  double lambda21_;
  double lambda22_;
  double rho2_1_;
  double rho2_2_;

  geometry_msgs::TwistStamped twist_;
  geometry_msgs::TwistStamped twist_cmd_;
  Eigen::Vector3d normalized_attitude_gain_;
  Eigen::Vector3d normalized_angular_rate_gain_;
  Eigen::MatrixX4d angular_acc_to_rotor_velocities_;
  Eigen::Vector3d angle_error_;
  Eigen::Vector3d angle_;

  mav_msgs::EigenTrajectoryPoint command_trajectory_;
  EigenOdometry odometry_;
  EigenOdometry leader_odometry_;
  std::ofstream f1_;
  EigenOdometry leader_desired_odometry_;

  void ComputeDesiredAngularAcc(const Eigen::Vector3d& acceleration,
                                Eigen::Vector3d* angular_acceleration,
                                double* thrust) ;
  void ComputeDesiredAcceleration(Eigen::Vector3d* acceleration) ;
};
}

#endif // ROTORS_CONTROL_SLIDING_MODE_CONTROLLER_H
