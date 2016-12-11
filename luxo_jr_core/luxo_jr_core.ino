/*******************************************************************************
* Copyright 2016 Luxo.Jr Team in OROCA.
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

#ifndef LUXO_CORE_INO
#define LUXO_CORE_INO

#include "luxo_jr_core_config.h"

HardwareTimer Timer(TIMER_CH1);
RC100 remote_controller;
WheelDriver LuxoJrWheel;
LuxoJrController LuxoJrJoint;

int luxo_jr_dxl_present_pos_[4] = {0, 0, 0, 0};
int luxo_jr_dxl_goal_pos_[4] = {0, 0, 0, 0}; //degree

float luxo_jr_dxl_present_rad_[4] = {0.0, 0.0, 0.0, 0.0};
float luxo_jr_dxl_goal_rad_[4] = {0.0, 0.0, 0.0, 0.0};

float computed_wheel_vel_[2] = {0.0, 0.0}, luxo_jr_wheel_vel_[2] = {0.0, 0.0};
float computed_joint_vel_[4] = {0.0, 0.0, 0.0, 0.0};

float luxo_jr_linear_x_ = 0.0, luxo_jr_angular_z_ = 0.0, const_cmd_vel_ = 0.0;
float ts_ = 0.008, luxo_jr_acc_ = 0.0, luxo_jr_max_vel_ = 0.0;
int mov_cnt_ = 0, scene_ = 1;

bool wheel_motion_flag_  = false;
bool luxo_jr_motion_flag_ = false;

float acceleration_;
float deceleration_;
float max_velocity_;
//
// float initial_pos_;
//
// float current_time_;
// float current_pos_;
// float current_vel_;
// float current_acc_;
//
// float final_pos_;
//
float accel_time_;
float const_time_;
float decel_time_;
//
float const_start_time_;
float decel_start_time_;
float move_time_;

void setup()
{
  Serial.begin(115200);
  while(!Serial);

  remote_controller.begin(1);

  LuxoJrWheel.init();
  LuxoJrJoint.init();

  //blink(1500, 3);
  timerInit();
}

void loop()
{
  //receiveRemoteControl();

  // Serial.print("luxo_jr_linear_x : ");
  // Serial.print(luxo_jr_linear_x);
  // Serial.print(" luxo_jr_angular_z : ");
  // Serial.print(luxo_jr_angular_z);
  //
  // for (int id = 1; id < 2; id++)
  // {
  //   LuxoJrJoint.readPosition(id, &luxo_jr_present_pos[id-1]);
  //   Serial.print(" luxo_jr_present_pos : ");
  //   Serial.print(luxo_jr_present_pos[0]);
  //   Serial.print(" ");
  //   Serial.print(luxo_jr_present_pos[1]);
  //   Serial.print(" ");
  //   Serial.print(luxo_jr_present_pos[2]);
  //   Serial.print(" ");
  //   Serial.println(luxo_jr_present_pos[3]);
  // }
}

void timerInit()
{
  Timer.pause();
  Timer.setPeriod(CONTROL_PEROID);           // in microseconds
  Timer.attachInterrupt(handler_control);
  Timer.refresh();
  Timer.resume();
}

void handler_control(void)
{
  // if (wheel_motion_flag_ == true)
  // {
  //   scene_++;
  //   wheel_motion_flag_ = false;
  // }
  //
  // trailor(scene_);
  //
  // controlMotorSpeed();

  for (int id = 1; id < MOTOR_NUM+1; id++)
  {
    LuxoJrJoint.readPosition(id, &luxo_jr_dxl_present_pos_[id-1]);
  }

  luxo_jr_dxl_goal_pos_[0] = -90;
  luxo_jr_dxl_goal_pos_[1] = 0;
  luxo_jr_dxl_goal_pos_[2] = 0;
  luxo_jr_dxl_goal_pos_[3] = 0;

  trapezoidalTimeProfile(luxo_jr_dxl_present_pos_, luxo_jr_dxl_goal_pos_, 0.3, 0.3, 2.0);
  luxoJrAction();
}

void trailor(int scene)
{
  switch (scene)
  {
    case 1:
      wheel_motion(-0.0, 0.1, 0.5, 0.5);
      break;
    case 2:
      wheel_motion(0.0, -0.1, 0.5, 0.5);
      break;
    default:
      wheel_motion(0.0, -0.1, 0.5, 0.5);
      break;
  }
}

void wheel_motion(float linear, float angular, float acc, float vel)
{
  float goal_vel[2];

  luxo_jr_wheel_vel_[0] = luxo_jr_linear_x_;
  luxo_jr_wheel_vel_[1] = luxo_jr_angular_z_;

  goal_vel[0]  = linear;
  goal_vel[1]  = angular;

  trapezoidalVelocityProfile(luxo_jr_wheel_vel_, goal_vel, acc, vel);
}

void receiveRemoteControl(void)
{
  int received_data = 0;

  if (remote_controller.available())
  {
    received_data = remote_controller.readData();

    if(received_data & RC100_BTN_U)
    {
      luxo_jr_linear_x_  += VELOCITY_LINEAR_X;
    }
    else if(received_data & RC100_BTN_D)
    {
      luxo_jr_linear_x_  -= VELOCITY_LINEAR_X;
    }
    else if(received_data & RC100_BTN_L)
    {
      luxo_jr_angular_z_ += VELOCITY_ANGULAR_Z;
    }
    else if(received_data & RC100_BTN_R)
    {
      luxo_jr_angular_z_ -= VELOCITY_ANGULAR_Z;
    }
    else if(received_data & RC100_BTN_1)
    {
      const_cmd_vel_ += VELOCITY_STEP;
    }
    else if(received_data & RC100_BTN_3)
    {
      const_cmd_vel_ -= VELOCITY_STEP;
    }
    else if(received_data & RC100_BTN_6)
    {
      luxo_jr_linear_x_  = const_cmd_vel_;
      luxo_jr_angular_z_ = 0.0;
    }
    else if(received_data & RC100_BTN_5)
    {
      luxo_jr_linear_x_  = 0.0;
      luxo_jr_angular_z_ = 0.0;
    }
  }
}

void controlMotorSpeed(void)
{
  double wheel_speed_cmd[2];
  double lin_vel1;
  double lin_vel2;

  wheel_speed_cmd[LEFT]  = luxo_jr_linear_x_ - (luxo_jr_angular_z_ * WHEEL_SEPARATION / 2);
  wheel_speed_cmd[RIGHT] = luxo_jr_linear_x_ + (luxo_jr_angular_z_ * WHEEL_SEPARATION / 2);

  lin_vel1 = wheel_speed_cmd[LEFT] * VELOCITY_CONSTANT_VAULE;

  if (lin_vel1 > LIMIT_XM_MAX_VELOCITY)       lin_vel1 =  LIMIT_XM_MAX_VELOCITY;
  else if (lin_vel1 < -LIMIT_XM_MAX_VELOCITY) lin_vel1 = -LIMIT_XM_MAX_VELOCITY;

  lin_vel2 = wheel_speed_cmd[RIGHT] * VELOCITY_CONSTANT_VAULE;

  if (lin_vel2 > LIMIT_XM_MAX_VELOCITY)       lin_vel2 =  LIMIT_XM_MAX_VELOCITY;
  else if (lin_vel2 < -LIMIT_XM_MAX_VELOCITY) lin_vel2 = -LIMIT_XM_MAX_VELOCITY;

  LuxoJrWheel.speedControl((int64_t)lin_vel1, (int64_t)lin_vel2);
}

void trapezoidalVelocityProfile(float pre_vel[2], float goal_vel[2], float acc, float max_vel)
{
  //Serial.println(luxo_jr_linear_x_);

  for (int num = 0; num < 2; num++)
  {
    if (goal_vel[num] > pre_vel[num])
    {
      computed_wheel_vel_[num] = min(computed_wheel_vel_[num] + (acc * ts_), max_vel);
      computed_wheel_vel_[num] = min(computed_wheel_vel_[num], sqrt(2*acc*abs(goal_vel[num] - luxo_jr_wheel_vel_[num])));
      luxo_jr_wheel_vel_[num] = luxo_jr_wheel_vel_[num] + computed_wheel_vel_[num] * ts_;

      luxo_jr_linear_x_  = luxo_jr_wheel_vel_[0];
      luxo_jr_angular_z_ = luxo_jr_wheel_vel_[1];
    }
    else if (goal_vel[num] < pre_vel[num])
    {
      computed_wheel_vel_[num] = max(computed_wheel_vel_[num] - (acc * ts_), -max_vel);
      computed_wheel_vel_[num] = max(computed_wheel_vel_[num], -sqrt(2*acc*abs(goal_vel[num] - luxo_jr_wheel_vel_[num])));
      luxo_jr_wheel_vel_[num] = luxo_jr_wheel_vel_[num] + computed_wheel_vel_[num] * ts_;

      luxo_jr_linear_x_  = luxo_jr_wheel_vel_[0];
      luxo_jr_angular_z_ = luxo_jr_wheel_vel_[1];
    }

    if ( abs(luxo_jr_wheel_vel_[0] - goal_vel[0]) < 0.001 && abs(luxo_jr_wheel_vel_[1] - goal_vel[1]) < 0.001 )
    {
      wheel_motion_flag_ = true;
    }
  }
}

void trapezoidalTimeProfile(int pre_pos[4], int goal_pos[4], float acc_time, float total_time)
{
  trapezoidalTimeProfile(pre_pos, goal_pos, acc_time, acc_time, total_time);
}

void trapezoidalTimeProfile(int pre_pos[4], int goal_pos[4], float acc_time, float decel_time, float total_time)
{
  if (mov_cnt_ != 0)
  {
    return;
  }
  else
  {
    luxo_jr_dxl_present_rad_[0] = LuxoJrJoint.convertValue2Radian(pre_pos[0]);
    luxo_jr_dxl_goal_rad_[0]   = goal_pos[0]*DEGREE2RADIAN;
    move_time_  = fabs(total_time);

    if((fabs(acc_time) + fabs(decel_time)) <= move_time_)
    {
      accel_time_ = fabs(acc_time);
      decel_time_ = fabs(decel_time);
      const_time_ = move_time_ - accel_time_ - decel_time_;
    }
    else
    {
      float time_gain = move_time_ / (fabs(acc_time) + fabs(decel_time));
      accel_time_ = time_gain*fabs(acc_time);
      decel_time_ = time_gain*fabs(decel_time);
      const_time_ = 0;
    }

    const_start_time_ = accel_time_;
    decel_start_time_ = accel_time_ + const_time_;

    float pos_diff = luxo_jr_dxl_goal_rad_[0] - luxo_jr_dxl_present_rad_[0];
    max_velocity_ = 2*pos_diff / (move_time_ + const_time_);
    acceleration_ = max_velocity_ / accel_time_;
    deceleration_ = -max_velocity_ / decel_time_;

    // Serial.print(move_time_);Serial.print("  ");
    // Serial.print(const_start_time_);Serial.print("  ");
    // Serial.print(decel_start_time_);Serial.print("  ");
    // Serial.print(max_velocity_);Serial.print("  ");
    // Serial.print(acceleration_);Serial.print("  ");
    // Serial.println(deceleration_);//Serial.print("  ");
    // //acc section
    // pos_coeff_accel_section_.coeffRef(0,0) = 0.5*acceleration_;
    // pos_coeff_accel_section_.coeffRef(1,0) = 0;
    // pos_coeff_accel_section_.coeffRef(2,0) = initial_pos_;
    //
    // vel_coeff_accel_section_.coeffRef(0,0) = 0;
    // vel_coeff_accel_section_.coeffRef(1,0) = acceleration_;
    // vel_coeff_accel_section_.coeffRef(2,0) = 0;
    //
    // //const section
    // pos_coeff_const_section_.coeffRef(0,0) = 0;
    // pos_coeff_const_section_.coeffRef(1,0) = max_velocity_;
    // pos_coeff_const_section_.coeffRef(2,0) = -0.5*acceleration_*accel_time_*accel_time_ + initial_pos_;
    //
    // vel_coeff_const_section_.coeffRef(0,0) = 0;
    // vel_coeff_const_section_.coeffRef(1,0) = 0;
    // vel_coeff_const_section_.coeffRef(2,0) = max_velocity_;
    //
    // //decel section
    // pos_coeff_decel_section_.coeffRef(0,0) = 0.5*deceleration_;
    // pos_coeff_decel_section_.coeffRef(1,0) = -deceleration_*move_time;
    // pos_coeff_decel_section_.coeffRef(2,0) = 0.5*deceleration_*move_time*move_time + final_pos_;
    //
    // vel_coeff_decel_section_.coeffRef(0,0) = 0;
    // vel_coeff_decel_section_.coeffRef(1,0) = deceleration_;
    // vel_coeff_decel_section_.coeffRef(2,0) = -deceleration_*move_time;
  }
}

void luxoJrAction()
{
  if (mov_cnt_ * ts_ < const_start_time_)
  {
    computed_joint_vel_[0] = computed_joint_vel_[0] + (acceleration_ * ts_);
    luxo_jr_dxl_present_rad_[0] = luxo_jr_dxl_present_rad_[0] + (computed_joint_vel_[0] * ts_);
    luxo_jr_dxl_present_pos_[0] = LuxoJrJoint.convertRadian2Value(luxo_jr_dxl_present_rad_[0]);

    LuxoJrJoint.positionControl(luxo_jr_dxl_present_pos_);
    mov_cnt_++;
    //Serial.print("accel");Serial.print("  ");
  }
  else if (mov_cnt_ * ts_ >= const_start_time_ && mov_cnt_ * ts_ < decel_start_time_)
  {
    computed_joint_vel_[0] = max_velocity_;
    luxo_jr_dxl_present_rad_[0] = luxo_jr_dxl_present_rad_[0] + (computed_joint_vel_[0] * ts_);
    luxo_jr_dxl_present_pos_[0] = LuxoJrJoint.convertRadian2Value(luxo_jr_dxl_present_rad_[0]);

    LuxoJrJoint.positionControl(luxo_jr_dxl_present_pos_);
    mov_cnt_++;
    //Serial.print("max");Serial.print("  ");
  }
  else if (mov_cnt_ <= move_time_ / ts_)
  {
    computed_joint_vel_[0] = computed_joint_vel_[0] + (deceleration_ * ts_);
    luxo_jr_dxl_present_rad_[0] = luxo_jr_dxl_present_rad_[0] + (computed_joint_vel_[0] * ts_);
    luxo_jr_dxl_present_pos_[0] = LuxoJrJoint.convertRadian2Value(luxo_jr_dxl_present_rad_[0]);

    LuxoJrJoint.positionControl(luxo_jr_dxl_present_pos_);
    mov_cnt_++;
    //Serial.print("decel");Serial.print("  ");
  }
  else //if ( abs(luxo_jr_dxl_present_rad_[0] - luxo_jr_dxl_goal_rad_[0]) < 0.001 )
  {
    mov_cnt_ = 0;
  }
  Serial.println(computed_joint_vel_[0]);Serial.print("  ");
  //Serial.print(luxo_jr_dxl_present_rad_[0]);Serial.print("  ");
  //Serial.print(luxo_jr_dxl_goal_rad_[0]);Serial.print("  ");
  //Serial.println(mov_cnt_);
}

void blink(int ms, int repeat)
{
  for (int num = 0; num < repeat; num++)
  {
    LuxoJrJoint.setLED(true);
    delay(ms);
    LuxoJrJoint.setLED(false);
    delay(ms);
  }
}
#endif // LUXO_CORE_INO
