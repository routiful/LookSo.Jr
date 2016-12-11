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

void setup()
{
  Serial.begin(115200);
  while(!Serial);

  remote_controller.begin(1);

  LuxoJrWheel.init();
  LuxoJrJoint.init();
#ifdef GET_MOTION
  for (int id = 1; id < MOTOR_NUM+1; id++)
  {
    LuxoJrJoint.setTorque(id, 0);
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
    LuxoJrJoint.readPosition(id, &luxo_jr_dxl_present_pos_[id-1]);
    luxo_jr_dxl_present_rad_[id-1] = LuxoJrJoint.convertValue2Radian(luxo_jr_dxl_present_pos_[id-1]);
  }

  Serial.print(" luxo_jr_get_degree : ");
  Serial.print(luxo_jr_dxl_present_rad_[0] * RADIAN2DEGREE);
  Serial.print(" ");
  Serial.print(luxo_jr_dxl_present_rad_[1] * RADIAN2DEGREE);
  Serial.print(" ");
  Serial.print(luxo_jr_dxl_present_rad_[2] * RADIAN2DEGREE);
  Serial.print(" ");
  Serial.println(luxo_jr_dxl_present_rad_[3] * RADIAN2DEGREE);
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
  if (luxo_jr_motion_end_flag_ == true && wheel_motion_end_flag_ == true)
  {
    luxo_mov_cnt_ = 0;
    wheel_mov_cnt_ = 0;

    wheel_motion_end_flag_ = false;
    luxo_jr_motion_end_flag_ = false;

    scene_++;
  }

  trailor(scene_);

  controlMotorSpeed();
#endif
}

void trailor(int scene)
{
  switch (scene)
  {
    case LUXO_JR_POWER_ON:
      luxo_jr_motion(21, 81, 30, 0, 0.3, 1.5);
      wheel_motion(0.0, 0.0, 0.0);
      break;
    case LUXO_JR_POWER_ON+STOP_MOTION:
      scene_delay(3.0);
      break;
    case HEAD_UP:
      luxo_jr_motion(21, 81, -45, 0, 0.3, 3.0);
      wheel_motion(0.0, 0.0, 0.0);
      break;
    case HEAD_UP+STOP_MOTION:
      scene_delay(1.0);
      break;
    case HEAD_TWIST:
      luxo_jr_motion(38, 80, -9, 0, 0.3, 1.0);
      wheel_motion(0.0, 0.0, 0.0);
      break;
    case HEAD_TWIST+STOP_MOTION:
      scene_delay(5.0);
      break;
    default:
      wheel_motion(0.0, 0.0, 0.0, 0.0);
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

void wheel_motion(float linear, float angular, float second)
{
  int delay_cnt = second / ts_;

  luxo_jr_linear_x_ = linear;
  luxo_jr_angular_z_ = angular;

  if (wheel_mov_cnt_ >= delay_cnt)
  {
    wheel_motion_end_flag_ = true;
    luxo_jr_linear_x_ = 0.0;
    luxo_jr_angular_z_ = 0.0;
  }
  else
  {
    wheel_motion_end_flag_ = false;
    wheel_mov_cnt_++;
  }
}

void luxo_jr_motion(int leg, int wrist, int neck, int head, float accel, float motion_time)
{
  if (luxo_jr_motion_end_flag_ == false)
  {
    for (int id = 1; id < MOTOR_NUM+1; id++)
    {
      LuxoJrJoint.readPosition(id, &luxo_jr_dxl_present_pos_[id-1]);
    }

    luxo_jr_dxl_goal_pos_[0] = leg;
    luxo_jr_dxl_goal_pos_[1] = wrist;
    luxo_jr_dxl_goal_pos_[2] = neck;
    luxo_jr_dxl_goal_pos_[3] = head;

    trapezoidalTimeProfile(luxo_jr_dxl_present_pos_, luxo_jr_dxl_goal_pos_, accel, motion_time);
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
  }

  if (luxo_jr_motion_end_flag_)//( abs(luxo_jr_wheel_vel_[0] - goal_vel[0]) < 0.001 && abs(luxo_jr_wheel_vel_[1] - goal_vel[1]) < 0.001 )
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
  if (luxo_mov_cnt_ != 0)
  {
    return;
  }
  else
  {
    for (int id = 0; id < MOTOR_NUM; id++)
    {
      luxo_jr_dxl_present_rad_[id] = LuxoJrJoint.convertValue2Radian(pre_pos[id]);
      luxo_jr_dxl_goal_rad_[id]   = goal_pos[id]*DEGREE2RADIAN;
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

      float pos_diff = luxo_jr_dxl_goal_rad_[id] - luxo_jr_dxl_present_rad_[id];
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
    if (luxo_mov_cnt_ * ts_ < const_start_time_[id])
    {
      computed_joint_vel_[id] = computed_joint_vel_[id] + (acceleration_[id] * ts_);
      luxo_jr_dxl_present_rad_[id] = luxo_jr_dxl_present_rad_[id] + (computed_joint_vel_[id] * ts_);
      luxo_jr_dxl_present_pos_[id] = LuxoJrJoint.convertRadian2Value(luxo_jr_dxl_present_rad_[id]);

      //LuxoJrJoint.positionControl(luxo_jr_dxl_present_pos_);
      //luxo_mov_cnt_[id]++;
      //Serial.print("accel");Serial.print("  ");
    }
    else if (luxo_mov_cnt_ * ts_ >= const_start_time_[id] && luxo_mov_cnt_ * ts_ < decel_start_time_[id])
    {
      computed_joint_vel_[id] = max_velocity_[id];
      luxo_jr_dxl_present_rad_[id] = luxo_jr_dxl_present_rad_[id] + (computed_joint_vel_[id] * ts_);
      luxo_jr_dxl_present_pos_[id] = LuxoJrJoint.convertRadian2Value(luxo_jr_dxl_present_rad_[id]);

      //LuxoJrJoint.positionControl(luxo_jr_dxl_present_pos_);
      //luxo_mov_cnt_[id]++;
      //Serial.print("max");Serial.print("  ");
    }
    else if (luxo_mov_cnt_* ts_ <= move_time_[id])
    {
      computed_joint_vel_[id] = computed_joint_vel_[id] + (deceleration_[id] * ts_);
      luxo_jr_dxl_present_rad_[id] = luxo_jr_dxl_present_rad_[id] + (computed_joint_vel_[id] * ts_);
      luxo_jr_dxl_present_pos_[id] = LuxoJrJoint.convertRadian2Value(luxo_jr_dxl_present_rad_[id]);

      //LuxoJrJoint.positionControl(luxo_jr_dxl_present_pos_);
      //luxo_mov_cnt_++;
      //Serial.print("decel");Serial.print("  ");
    }
    else
    {
      luxo_jr_motion_end_flag_ = true;
    }
    //Serial.println(computed_joint_vel_[0]);Serial.print("  ");
    //Serial.print(luxo_jr_dxl_present_rad_[0]);Serial.print("  ");
    //Serial.print(luxo_jr_dxl_goal_rad_[0]);Serial.print("  ");
    //Serial.println(luxo_mov_cnt_);
  }
  luxo_mov_cnt_++;
  LuxoJrJoint.positionControl(luxo_jr_dxl_present_pos_);
}

void blink(float second, int repeat)
{
  for (int num = 0; num < repeat; num++)
  {
    LuxoJrJoint.setLED(true);
    delay(second*1000);
    LuxoJrJoint.setLED(false);
    delay(second*1000);
  }
}

void scene_delay(float second)
{
  int delay_cnt = second / ts_;

  if (scene_delay_cnt >= delay_cnt)
  {
    scene_delay_end_flag_ = true;
    scene_++;
  }
  else
  {
    scene_delay_end_flag_ = false;
    scene_delay_cnt++;
  }
}
#endif // LUXO_CORE_INO
