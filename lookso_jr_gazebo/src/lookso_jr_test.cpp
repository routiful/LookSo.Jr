/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim */


#include <ros/ros.h>
#include <math.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#define DEGREE2RADIAN (M_PI / 180.0)
#define RADIAN2DEGREE (180.0 / M_PI)

ros::Publisher lookso_jr_joint_state_pub_;

void looksoJointMsgCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std_msgs::Float64 joint;

  joint.data = 30 * DEGREE2RADIAN;

  lookso_jr_joint_state_pub_.publish(joint);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lookso_jr_test");
  ros::NodeHandle nh("~");

  lookso_jr_joint_state_pub_ = nh.advertise<sensor_msgs::JointState>("/joint_states", 10);

  ros::Subscriber lookso_jr_joint_state_sub = nh.subscribe("/joint_states", 10, looksoJointMsgCallback);

  ros::spin();

  return 0;
}
