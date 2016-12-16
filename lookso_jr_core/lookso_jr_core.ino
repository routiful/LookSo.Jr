/*******************************************************************************
* Copyright 2016 LookSo.Jr Team in OROCA.
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
#ifndef LOOKSO_CORE_INO
#define LOOKSO_CORE_INO

#include "lookso_jr_core_config.h"

HardwareTimer Timer(TIMER_CH1);
RC100 remote_controller;
WheelDriver LooksoJrWheel;
LookSoJrController LooksoJrJoint;

void setup()
{
  Serial.begin(115200);
  //while(!Serial);

  remote_controller.begin(1);

  LooksoJrWheel.init();
  LooksoJrJoint.init();
#ifdef GET_MOTION
  for (int id = 1; id < MOTOR_NUM+1; id++)
  {
    LooksoJrJoint.setTorque(id, 0);
  }
#endif

  //blink(1.5, 3);
  timerInit();
}

void loop()
{
  //receiveRemoteControl();
#ifdef GET_MOTION
  for (int id = 1; id < MOTOR_NUM+1; id++)
  {
    LooksoJrJoint.readPosition(id, &lookso_jr_dxl_present_pos_[id-1]);
    lookso_jr_dxl_present_rad_[id-1] = LooksoJrJoint.convertValue2Radian(lookso_jr_dxl_present_pos_[id-1]);
  }

  Serial.print(" lookso_jr_get_degree : ");
  Serial.print(lookso_jr_dxl_present_rad_[0] * RADIAN2DEGREE);
  Serial.print(" ");
  Serial.print(lookso_jr_dxl_present_rad_[1] * RADIAN2DEGREE);
  Serial.print(" ");
  Serial.print(lookso_jr_dxl_present_rad_[2] * RADIAN2DEGREE);
  Serial.print(" ");
  Serial.println(lookso_jr_dxl_present_rad_[3] * RADIAN2DEGREE);
#endif
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
#ifdef MOTION_PLAY
  if (lookso_jr_motion_end_flag_ == true && wheel_motion_end_flag_ == true)
  {
    if (scene_delay_end_flag_ == false)
    {
      scene_delay(trailor_[scene_][8]);
    }
    else if (scene_delay_end_flag_ == true)
    {
      lookso_mov_cnt_ = 0;
      wheel_mov_cnt_ = 0;
      scene_delay_cnt_ = 0;

      wheel_motion_end_flag_ = false;
      lookso_jr_motion_end_flag_ = false;
      scene_delay_end_flag_ = false;

      if (scene_ == scene_cnt_)
      {
        scene_ = scene_cnt_;
      }
      else
      {
        scene_++;
      }
    }
  }
  else
  {
    lookso_jr_motion(trailor_[scene_][0], trailor_[scene_][1], trailor_[scene_][2], trailor_[scene_][3], 0.3, trailor_[scene_][4]);
    wheel_motion(trailor_[scene_][5], trailor_[scene_][6], trailor_[scene_][7]);

    controlMotorSpeed();
  }
#endif
}

void wheel_motion(float linear, float angular, float acc, float vel)
{
  float goal_vel[2];

  lookso_jr_wheel_vel_[0] = lookso_jr_linear_x_;
  lookso_jr_wheel_vel_[1] = lookso_jr_angular_z_;

  goal_vel[0]  = linear;
  goal_vel[1]  = angular;

  trapezoidalVelocityProfile(lookso_jr_wheel_vel_, goal_vel, acc, vel);
}

void wheel_motion(float linear, float angular, float second)
{
  int delay_cnt = second / ts_;

  lookso_jr_linear_x_ = linear;
  lookso_jr_angular_z_ = angular;

  if (wheel_mov_cnt_ >= delay_cnt)
  {
    wheel_motion_end_flag_ = true;
    lookso_jr_linear_x_ = 0.0;
    lookso_jr_angular_z_ = 0.0;
  }
  else
  {
    wheel_motion_end_flag_ = false;
    wheel_mov_cnt_++;
  }
}

void lookso_jr_motion(int leg, int wrist, int neck, int head, float accel, float motion_time)
{
  if (lookso_jr_motion_end_flag_ == false)
  {
    for (int id = 1; id < MOTOR_NUM+1; id++)
    {
      LooksoJrJoint.readPosition(id, &lookso_jr_dxl_present_pos_[id-1]);
    }

    lookso_jr_dxl_goal_pos_[0] = leg;
    lookso_jr_dxl_goal_pos_[1] = wrist;
    lookso_jr_dxl_goal_pos_[2] = neck;
    lookso_jr_dxl_goal_pos_[3] = head;

    trapezoidalTimeProfile(lookso_jr_dxl_present_pos_, lookso_jr_dxl_goal_pos_, accel, motion_time);
    luxoJrAction();
  }

}
void receiveRemoteControl(void)
{
  int received_data = 0;

  if (remote_controller.available())
  {
    received_data = remote_controller.readData();

    if(received_data & RC100_BTN_U)
    {
      lookso_jr_linear_x_  += VELOCITY_LINEAR_X;
    }
    else if(received_data & RC100_BTN_D)
    {
      lookso_jr_linear_x_  -= VELOCITY_LINEAR_X;
    }
    else if(received_data & RC100_BTN_L)
    {
      lookso_jr_angular_z_ += VELOCITY_ANGULAR_Z;
    }
    else if(received_data & RC100_BTN_R)
    {
      lookso_jr_angular_z_ -= VELOCITY_ANGULAR_Z;
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
      lookso_jr_linear_x_  = const_cmd_vel_;
      lookso_jr_angular_z_ = 0.0;
    }
    else if(received_data & RC100_BTN_5)
    {
      lookso_jr_linear_x_  = 0.0;
      lookso_jr_angular_z_ = 0.0;
    }
  }
}

void controlMotorSpeed(void)
{
  double wheel_speed_cmd[2];
  double lin_vel1;
  double lin_vel2;

  wheel_speed_cmd[LEFT]  = lookso_jr_linear_x_ - (lookso_jr_angular_z_ * WHEEL_SEPARATION / 2);
  wheel_speed_cmd[RIGHT] = lookso_jr_linear_x_ + (lookso_jr_angular_z_ * WHEEL_SEPARATION / 2);

  lin_vel1 = wheel_speed_cmd[LEFT] * VELOCITY_CONSTANT_VAULE;

  if (lin_vel1 > LIMIT_XM_MAX_VELOCITY)       lin_vel1 =  LIMIT_XM_MAX_VELOCITY;
  else if (lin_vel1 < -LIMIT_XM_MAX_VELOCITY) lin_vel1 = -LIMIT_XM_MAX_VELOCITY;

  lin_vel2 = wheel_speed_cmd[RIGHT] * VELOCITY_CONSTANT_VAULE;

  if (lin_vel2 > LIMIT_XM_MAX_VELOCITY)       lin_vel2 =  LIMIT_XM_MAX_VELOCITY;
  else if (lin_vel2 < -LIMIT_XM_MAX_VELOCITY) lin_vel2 = -LIMIT_XM_MAX_VELOCITY;

  LooksoJrWheel.speedControl((int64_t)lin_vel1, (int64_t)lin_vel2);
}

void trapezoidalVelocityProfile(float pre_vel[2], float goal_vel[2], float acc, float max_vel)
{
  for (int num = 0; num < 2; num++)
  {
    if (goal_vel[num] > pre_vel[num])
    {
      computed_wheel_vel_[num] = min(computed_wheel_vel_[num] + (acc * ts_), max_vel);
      computed_wheel_vel_[num] = min(computed_wheel_vel_[num], sqrt(2*acc*abs(goal_vel[num] - lookso_jr_wheel_vel_[num])));
      lookso_jr_wheel_vel_[num] = lookso_jr_wheel_vel_[num] + computed_wheel_vel_[num] * ts_;

      lookso_jr_linear_x_  = lookso_jr_wheel_vel_[0];
      lookso_jr_angular_z_ = lookso_jr_wheel_vel_[1];
    }
    else if (goal_vel[num] < pre_vel[num])
    {
      computed_wheel_vel_[num] = max(computed_wheel_vel_[num] - (acc * ts_), -max_vel);
      computed_wheel_vel_[num] = max(computed_wheel_vel_[num], -sqrt(2*acc*abs(goal_vel[num] - lookso_jr_wheel_vel_[num])));
      lookso_jr_wheel_vel_[num] = lookso_jr_wheel_vel_[num] + computed_wheel_vel_[num] * ts_;

      lookso_jr_linear_x_  = lookso_jr_wheel_vel_[0];
      lookso_jr_angular_z_ = lookso_jr_wheel_vel_[1];
    }
  }

  if (lookso_jr_motion_end_flag_)
  {
    wheel_motion_end_flag_ = true;
  }
}

void trapezoidalTimeProfile(int pre_pos[4], int goal_pos[4], float acc_time, float total_time)
{
  trapezoidalTimeProfile(pre_pos, goal_pos, acc_time, acc_time, total_time);
}

void trapezoidalTimeProfile(int pre_pos[4], int goal_pos[4], float acc_time, float decel_time, float total_time)
{
  if (lookso_mov_cnt_ != 0)
  {
    return;
  }
  else
  {
    for (int id = 0; id < MOTOR_NUM; id++)
    {
      lookso_jr_dxl_present_rad_[id] = LooksoJrJoint.convertValue2Radian(pre_pos[id]);
      lookso_jr_dxl_goal_rad_[id]   = goal_pos[id]*DEGREE2RADIAN;
      move_time_[id]  = fabs(total_time);

      if((fabs(acc_time) + fabs(decel_time)) <= move_time_[id])
      {
        accel_time_[id] = fabs(acc_time);
        decel_time_[id] = fabs(decel_time);
        const_time_[id] = move_time_[id] - accel_time_[id] - decel_time_[id];
      }
      else
      {
        float time_gain = move_time_[id] / (fabs(acc_time) + fabs(decel_time));
        accel_time_[id] = time_gain*fabs(acc_time);
        decel_time_[id] = time_gain*fabs(decel_time);
        const_time_[id] = 0;
      }

      const_start_time_[id] = accel_time_[id];
      decel_start_time_[id] = accel_time_[id] + const_time_[id];

      float pos_diff = lookso_jr_dxl_goal_rad_[id] - lookso_jr_dxl_present_rad_[id];
      max_velocity_[id] = 2*pos_diff / (move_time_[id] + const_time_[id]);
      acceleration_[id] = max_velocity_[id] / accel_time_[id];
      deceleration_[id] = -max_velocity_[id] / decel_time_[id];
    }
  }
}

void luxoJrAction()
{
  for (int id = 0; id < MOTOR_NUM; id++)
  {
    if (lookso_mov_cnt_ * ts_ < const_start_time_[id])
    {
      computed_joint_vel_[id] = computed_joint_vel_[id] + (acceleration_[id] * ts_);
      lookso_jr_dxl_present_rad_[id] = lookso_jr_dxl_present_rad_[id] + (computed_joint_vel_[id] * ts_);
      lookso_jr_dxl_present_pos_[id] = LooksoJrJoint.convertRadian2Value(lookso_jr_dxl_present_rad_[id]);
    }
    else if (lookso_mov_cnt_ * ts_ >= const_start_time_[id] && lookso_mov_cnt_ * ts_ < decel_start_time_[id])
    {
      computed_joint_vel_[id] = max_velocity_[id];
      lookso_jr_dxl_present_rad_[id] = lookso_jr_dxl_present_rad_[id] + (computed_joint_vel_[id] * ts_);
      lookso_jr_dxl_present_pos_[id] = LooksoJrJoint.convertRadian2Value(lookso_jr_dxl_present_rad_[id]);
    }
    else if (lookso_mov_cnt_* ts_ <= move_time_[id])
    {
      computed_joint_vel_[id] = computed_joint_vel_[id] + (deceleration_[id] * ts_);
      lookso_jr_dxl_present_rad_[id] = lookso_jr_dxl_present_rad_[id] + (computed_joint_vel_[id] * ts_);
      lookso_jr_dxl_present_pos_[id] = LooksoJrJoint.convertRadian2Value(lookso_jr_dxl_present_rad_[id]);
    }
    else
    {
      lookso_jr_motion_end_flag_ = true;
    }
  }
  lookso_mov_cnt_++;
  LooksoJrJoint.positionControl(lookso_jr_dxl_present_pos_);
}

void blink(float second, int repeat)
{
  for (int num = 0; num < repeat; num++)
  {
    LooksoJrJoint.setLED(true);
    delay(second*1000);
    LooksoJrJoint.setLED(false);
    delay(second*1000);
  }
}

void scene_delay(float second)
{
  int delay_cnt = second / ts_;

  if (scene_delay_cnt_ >= delay_cnt)
  {
    scene_delay_end_flag_ = true;
  }
  else
  {
    scene_delay_end_flag_ = false;
    scene_delay_cnt_++;
  }
}
#endif // LOOKSO_CORE_INO
