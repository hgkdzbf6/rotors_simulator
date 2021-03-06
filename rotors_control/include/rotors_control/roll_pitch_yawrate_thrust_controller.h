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

#ifndef ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CONTROLLER_H
#define ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CONTROLLER_H

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>

#include "rotors_control/common.h"
#include "rotors_control/parameters.h"

namespace rotors_control {

// Default values for the roll pitch yawrate thrust controller and the Asctec Firefly.
static const Eigen::Vector3d kDefaultAttitudeGain = Eigen::Vector3d(3, 3, 0.035);
static const Eigen::Vector3d kDefaultAngularRateGain = Eigen::Vector3d(0.52, 0.52, 0.025);

class RollPitchYawrateThrustControllerParameters {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  RollPitchYawrateThrustControllerParameters()
      :k1(9),k2(18),lambda1(5),lambda2(4),rho(0.9), 
      attitude_gain_(kDefaultAttitudeGain),
        angular_rate_gain_(kDefaultAngularRateGain) {
          rho_1 = rho / (2-rho);
          rho_2 = rho;
    calculateAllocationMatrix(rotor_configuration_, &allocation_matrix_);
  }

  double k1;
  double k2;
  double lambda1;
  double lambda2;
  double rho;
  double rho_1;
  double rho_2;
  Eigen::Matrix4Xd allocation_matrix_;
  Eigen::Vector3d attitude_gain_;
  Eigen::Vector3d angular_rate_gain_;
  RotorConfiguration rotor_configuration_;
};

class RollPitchYawrateThrustController {
 public:
  RollPitchYawrateThrustController();
  ~RollPitchYawrateThrustController();
  void InitializeParameters();
  void CalculateRotorVelocities(Eigen::VectorXd* rotor_velocities) ;
  void ComputeDesiredAcceleration(Eigen::Vector3d* acceleration);
  void SetOdometry(const EigenOdometry& odometry);
  void SetRollPitchYawrateThrust(
      const mav_msgs::EigenRollPitchYawrateThrust& roll_pitch_yawrate_thrust);
  void SetTrajectoryPoint(
    const mav_msgs::EigenTrajectoryPoint& command_trajectory);

  void SetEnable(bool val);
  void SetTakingoff(bool val);
  bool GetTakingoff();
  RollPitchYawrateThrustControllerParameters controller_parameters_;
  VehicleParameters vehicle_parameters_;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 private:
  bool initialized_params_;
  bool controller_active_;
  bool taking_off_;

  double sign(double input);

  Eigen::Vector3d s1_int_;
  Eigen::Vector3d s2_int_;
  Eigen::Vector3d u_t4_int_;

  double k1_;
  double k2_;
  double lambda1_;
  double lambda2_;
  double rho_1_;
  double rho_2_;

  mav_msgs::EigenTrajectoryPoint command_trajectory_;

  Eigen::Vector3d normalized_attitude_gain_;
  Eigen::Vector3d normalized_angular_rate_gain_;
  Eigen::MatrixX4d angular_acc_to_rotor_velocities_;

  mav_msgs::EigenRollPitchYawrateThrust roll_pitch_yawrate_thrust_;
  EigenOdometry odometry_;

  void ComputeDesiredAngularAcc(Eigen::Vector3d acceleration, Eigen::Vector3d* angular_acceleration) const;
};
}

#endif // ROTORS_CONTROL_ROLL_PITCH_YAWRATE_THRUST_CONTROLLER_H
