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

#ifndef LOOKSO_CORE_CONFIG_H_
#define LOOKSO_CORE_CONFIG_H_

// #include <IMU.h>
#include <RC100.h>
#include <math.h>

#include "wheel_driver.h"
#include "lookso_jr_controller.h"

#define CONTROL_PEROID              8000

#define LEFT                        0
#define RIGHT                       1

#define WHEEL_RADIUS                0.033 // radian
#define WHEEL_SEPARATION            0.16  // meter

#define VELOCITY_CONSTANT_VAULE     1263.632956882  // V = r * w = r * RPM * 0.10472
                                                     //   = 0.033 * 0.229 * Goal RPM * 0.10472
                                                     // Goal RPM = V * 1263.632956882

#define VELOCITY_LINEAR_X           0.05
#define VELOCITY_ANGULAR_Z          0.05
#define VELOCITY_STEP               0.02

#define LIMIT_XM_MAX_VELOCITY       480

#define MOTOR_NUM                   4

#define DEGREE2RADIAN               (PI / 180.0)
#define RADIAN2DEGREE               (180.0 / PI)

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)<(b)?(b):(a))

// #define GET_MOTION
 #define MOTION_PLAY

int lookso_jr_dxl_present_pos_[4] = {0, 0, 0, 0};
int lookso_jr_dxl_goal_pos_[4] = {0, 0, 0, 0}; //degree

float lookso_jr_dxl_present_rad_[4] = {0.0, 0.0, 0.0, 0.0};
float lookso_jr_dxl_goal_rad_[4] = {0.0, 0.0, 0.0, 0.0};

float computed_wheel_vel_[2] = {0.0, 0.0}, lookso_jr_wheel_vel_[2] = {0.0, 0.0};
float computed_joint_vel_[4] = {0.0, 0.0, 0.0, 0.0};

float lookso_jr_linear_x_ = 0.0, lookso_jr_angular_z_ = 0.0, const_cmd_vel_ = 0.0;
float ts_ = 0.008, lookso_jr_acc_ = 0.0, lookso_jr_max_vel_ = 0.0;

float acceleration_[4] = {0.0, 0.0, 0.0, 0.0};
float deceleration_[4] = {0.0, 0.0, 0.0, 0.0};
float max_velocity_[4] = {0.0, 0.0, 0.0, 0.0};

float accel_time_[4] = {0.0, 0.0, 0.0, 0.0};
float const_time_[4] = {0.0, 0.0, 0.0, 0.0};
float decel_time_[4] = {0.0, 0.0, 0.0, 0.0};

float const_start_time_[4] = {0.0, 0.0, 0.0, 0.0};
float decel_start_time_[4] = {0.0, 0.0, 0.0, 0.0};
float move_time_[4] = {0.0, 0.0, 0.0, 0.0};

int lookso_mov_cnt_ = 0, wheel_mov_cnt_ = 0, scene_delay_cnt_ = 0;

bool wheel_motion_end_flag_  = false;
bool lookso_jr_motion_end_flag_ = false;
bool scene_delay_end_flag_ = false;

int scene_ = 0, scene_cnt_ = 124;

// leg, wrist, neck, head, motion_time, linear, angular, second, delay
float trailor_[125][9] = {
  //#1 Please move(1~5)
  {21, 81, 30, 0,     1.5,   0.0, 0.0, 0.0,    3.0},     //lookso_jr_POWER_ON
  {21, 81, -45, -90,  2.0,   0.0, 0.0, 0.0,    3.0},     //HEAD_UP
  {10,-10, 10, 0,     0.6,   0.0, 0.0, 0.0,    0.5},     //STRETCH
  {21, 81, 30, -90,   3.0,   0.0, 0.0, 0.0,    3.0},     //HEAD_DOWN
  {21, 81, 30, -90,   0.2,   0.0,-3.0, 0.2,    0.1},     //SPIN
  {21, 81, 30, -90,   0.2,   0.0, 3.0, 0.2,    0.1},     //SPIN
  {21, 81, 30, -90,   0.2,   0.0,-3.0, 0.2,    0.1},     //SPIN
  {21, 81, 30, -90,   0.2,   0.0, 3.0, 0.2,    0.1},     //SPIN
  {21, 81, 30, -90,   0.2,   0.0,-3.0, 0.2,    0.1},     //SPIN
  {21, 81, 30, -90,   0.2,   0.0, 3.0, 0.2,    0.1},     //SPIN
  {21, 81, 30, -90,   0.2,   0.0,-3.0, 0.2,    0.1},     //SPIN
  {21, 81, 30, -90,   0.2,   0.0, 3.0, 0.2,    0.1},     //SPIN
  {21, 81, 30, -90,   0.2,   0.0,-3.0, 0.2,    38.0},     //SPIN
  // #2 Luxo Jr wake up(6~23)
  {21, 81, 30, -70,   0.1,   0.0, 0.0, 0.0,    1.0},     //HEAD_TWIST
  {21, 81, 30, -90,   0.1,   0.0, 0.0, 0.0,    5.0},     //HEAD_TWIST
  {-41, 63, 61,  0,   3.0,   0.0, 0.0, 0.0,    1.0},     //GET_UP
  {-41, 63, 61, 20,   0.1,   0.0, 0.0, 0.0,    1.0},     //HEAD_TWIST_AGAIN
  {-41, 63, 61,  0,   0.1,   0.0, 0.0, 0.0,    2.0},     //HEAD_TWIST_AGAIN
  {-41, 63, 30,  0,   1.0,   0.0, 0.0, 0.0,    1.0},     //LOOK_UP
  {-41, 63, 90,  0,   2.0,   0.0, 0.0, 0.0,    1.0},     //LOOK_DOWN
  {-41, 63, 61,  0,   1.0,   0.0, 0.0, 0.0,    1.0},     //LOOK_FORWARD
  {-6,  54, 36,  0,   0.3,   2.0, 0.0, 0.3,    1.0},     //MOVE_FORWARD
  {-6,  54, 36, 30,   0.2,   0.0, 0.0, 0.0,    2.0},     //HEAD_TWIST_AGAIN_AGAIN
  {-41, 63, 61, 30,   0.3,  -1.5, 0.0, 0.1,    2.0},     //MOVE_BACK
  {-41, 63, 61,  0,   0.5,   0.0,-0.5, 4.0,    0.2},     //LOOK_AROUND
  {-80, 70, 81,  0,   0.2,   0.0, 0.0, 0.0,    2.0},     //SURPRISE
  {-41, 63, 90,  0,   1.0,   0.0, 0.0, 0.0,    0.3},     //SCAN_DOWN
  {-41, 63, 40,  0,   2.0,   0.0, 0.0, 0.0,    0.3},     //SCAN_UP
  {-41, 63, 61,  0,   1.0,   0.0, 0.0, 0.0,    0.3},     //SCAN_FORWARD
  {-41, 63, 61,  0,   0.7,   0.0,-0.5, 0.7,    2.0},     //LOOK_AROUND
  {-41, 63, 61, 0,   3.0,   0.0, 0.0, 0.0,    20.0},     //HEAD_TWIST_LAST
  // #3 Dance time(24~)
  {-41, 63, 70,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //HEAD_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    1.0},     //HEAD_SHAKE
  {-41, 63, 70,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //HEAD_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    1.0},     //HEAD_SHAKE
  {-41, 63, 70,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //HEAD_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    1.0},     //HEAD_SHAKE
  {-41, 63, 70,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //HEAD_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    1.0},     //HEAD_SHAKE
  {-41, 63, 70,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //HEAD_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //HEAD_SHAKE
  {-41, 63, 70,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //HEAD_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    1.0},     //HEAD_SHAKE
  {-41, 63, 70,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //HEAD_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //HEAD_SHAKE
  {-41, 63, 70,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //HEAD_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //HEAD_SHAKE
  {-41, 63, 70,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //HEAD_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.5},     //HEAD_SHAKE
  {-41, 63, 70,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //HEAD_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //HEAD_SHAKE
  {-41, 63, 70,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //HEAD_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //HEAD_SHAKE
  {-41, 63, 70,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //HEAD_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //HEAD_SHAKE
  {-41, 46, 80,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //WRIST_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //WRIST_SHAKE
  {-41, 46, 80,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //WRIST_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //WRIST_SHAKE
  {-41, 46, 80,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //WRIST_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //WRIST_SHAKE
  {-41, 46, 80,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //WRIST_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //WRIST_SHAKE
  {-41, 46, 80,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //WRIST_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //WRIST_SHAKE
  {-41, 46, 80,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //WRIST_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //WRIST_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0, 2.0, 0.3,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0,-2.0, 0.3,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0, 2.0, 0.3,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0,-2.0, 0.3,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0, 2.0, 0.3,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0,-2.0, 0.3,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0, 2.0, 0.3,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0,-2.0, 0.3,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 0.0, 0.0,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0,-2.0, 0.3,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 2.0, 0.3,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0,-2.0, 0.3,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 2.0, 0.3,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0,-2.0, 0.3,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 2.0, 0.3,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0,-2.0, 0.3,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 2.0, 0.3,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0,-2.0, 0.3,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 2.0, 0.3,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0,-2.0, 0.3,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0, 2.0, 0.3,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0, 2.0, 0.3,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0,-2.0, 0.3,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0, 2.0, 0.3,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0,-2.0, 0.3,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0, 2.0, 0.3,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0,-2.0, 0.3,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0, 2.0, 0.3,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0,-2.0, 0.3,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0, 2.0, 0.3,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0,-2.0, 0.3,    0.0},     //LEG_SHAKE
  {-70, 94, 68,-30,   0.5,   0.0, 2.0, 0.3,    0.0},     //LEG_SHAKE
  {-41, 63, 61,-30,   0.5,   0.0,-2.0, 0.3,    13.0},    //LEG_SHAKE
  {-41, 63, 61,-50,   0.1,   0.0, 0.0, 0.1,    0.5},     //HEAD_TWIST
  {-41, 63, 61,-30,   0.1,   0.0, 0.0, 0.1,    30.0}     //HEAD_TWIST
};

#endif // LOOKSO_CORE_CONFIG_H_
