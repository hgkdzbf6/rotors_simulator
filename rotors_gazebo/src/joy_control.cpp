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

#include <thread>
#include <chrono>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <tf/tf.h>

/**
* 功能: 把joy产生的位置信息变成轨迹信息,供飞机下一个时刻使用.
*/
class JoyControl{
  private:
    ros::NodeHandle nh_;  
    ros::Subscriber pose_sub_;
    ros::Publisher trajectory_pub_;
    geometry_msgs::PoseStamped pose_;
    ros::Timer timer_;
    bool is_real_;

    void TimerCallback(const ros::TimerEvent & e){
      // Wait for 5 seconds to let the Gazebo GUI show up.

      trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
      trajectory_msg.header.stamp = ros::Time::now();

      // Default desired position and yaw.
      Eigen::Vector3d desired_position(0.0, 0.0, 1.0);
      double roll,pitch,desired_yaw = 0.0;
      desired_position.x()=pose_.pose.position.x;
      desired_position.y()=pose_.pose.position.y;
      desired_position.z()=pose_.pose.position.z;

      // Overwrite defaults if set as node parameters.
      tf::Quaternion q(pose_.pose.orientation.x,
        pose_.pose.orientation.y,
        pose_.pose.orientation.z,
        pose_.pose.orientation.w);
      tf::Matrix3x3(q).getRPY(roll,pitch,desired_yaw);

      mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(
        desired_position, desired_yaw, &trajectory_msg);
      //ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
          // nh_.getNamespace().c_str(), desired_position.x(),
          // desired_position.y(), desired_position.z());
      trajectory_pub_.publish(trajectory_msg);	
    }

    void PoseCallback(const geometry_msgs::PoseStampedConstPtr & msg){
      pose_=*msg;
    }

  public:
    // JoyControl():pose_sub_(nh_.subscribe("command_pose",10,&JoyControl::PoseCallback,this)),
    JoyControl():pose_sub_(nh_.subscribe("filtered_pose",10,&JoyControl::PoseCallback,this)),
    trajectory_pub_(nh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>(mav_msgs::default_topics::COMMAND_TRAJECTORY, 10)){
      ros::NodeHandle nh_private("~");
      ROS_INFO("Started joy control");
      nh_private.param<bool>("is_real",is_real_,10);
      // Trying to unpause Gazebo for 10 seconds.
      std_srvs::Empty srv;
      bool unpaused;
      if(!is_real_){
        unsigned int i = 0;
        unpaused = ros::service::call("/gazebo/unpause_physics", srv);
        while (i <= 10 && !unpaused) {
          ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
          std::this_thread::sleep_for(std::chrono::seconds(1));
          unpaused = ros::service::call("/gazebo/unpause_physics", srv);
          ++i;
        }
      }
      timer_=nh_.createTimer(ros::Duration(0.01),&JoyControl::TimerCallback,this);
      if(!is_real_){
        if (!unpaused) {
          ROS_FATAL("Could not wake up Gazebo.");
          return ;
        } else {
          ROS_INFO("Unpaused the Gazebo simulation.");
        }
      }
      ros::spin();
    }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "joy_control");
  // Create a private node handle for accessing node parameters.
  JoyControl joy_control;
  ros::shutdown();
  return 0;
}
