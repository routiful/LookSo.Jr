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

#ifndef LUXO_CORE_CONFIG_H_
#define LUXO_CORE_CONFIG_H_

#include <IMU.h>
#include <RC100.h>
#include <math.h>

#include "wheel_driver.h"
#include "luxo_jr_controller.h"

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

// MOTION SEQUENCE
#define STOP_MOTION                 1
// #1 Please move
#define A_LUXO_JR_POWER_ON          1
#define A_HEAD_UP                   3
#define A_STRETCH                   5
#define A_HEAD_DOWN                 7
// #2 Luxo Jr wake up
#define B_HEAD_TWIST_GO             9
#define B_HEAD_TWIST_BACK           11
#define B_GET_UP                    13
#define B_HEAD_TWIST_AGAIN_GO       15
#define B_HEAD_TWIST_AGAIN_BACK     17
#define B_LOOK_UP                   19
#define B_LOOK_DOWN                 21
#define B_LOOK_FRONT                23
#define B_MOVE_FRONT                25
#define B_HEAD_TWIST_AGAIN          27
#define B_MOVE_BACK                 29
#define B_LOOK_AROUND_GO            31
#define B_LOOK_AROUND_BACK          33
#define B_HEAD_TWIST_LAST           35

int luxo_jr_dxl_present_pos_[4] = {0, 0, 0, 0};
int luxo_jr_dxl_goal_pos_[4] = {0, 0, 0, 0}; //degree

float luxo_jr_dxl_present_rad_[4] = {0.0, 0.0, 0.0, 0.0};
float luxo_jr_dxl_goal_rad_[4] = {0.0, 0.0, 0.0, 0.0};

float computed_wheel_vel_[2] = {0.0, 0.0}, luxo_jr_wheel_vel_[2] = {0.0, 0.0};
float computed_joint_vel_[4] = {0.0, 0.0, 0.0, 0.0};

float luxo_jr_linear_x_ = 0.0, luxo_jr_angular_z_ = 0.0, const_cmd_vel_ = 0.0;
float ts_ = 0.008, luxo_jr_acc_ = 0.0, luxo_jr_max_vel_ = 0.0;

float acceleration_[4] = {0.0, 0.0, 0.0, 0.0};
float deceleration_[4] = {0.0, 0.0, 0.0, 0.0};
float max_velocity_[4] = {0.0, 0.0, 0.0, 0.0};

float accel_time_[4] = {0.0, 0.0, 0.0, 0.0};
float const_time_[4] = {0.0, 0.0, 0.0, 0.0};
float decel_time_[4] = {0.0, 0.0, 0.0, 0.0};

float const_start_time_[4] = {0.0, 0.0, 0.0, 0.0};
float decel_start_time_[4] = {0.0, 0.0, 0.0, 0.0};
float move_time_[4] = {0.0, 0.0, 0.0, 0.0};

int luxo_mov_cnt_ = 0, wheel_mov_cnt_ = 0, scene_delay_cnt_ = 0;

bool wheel_motion_end_flag_  = false;
bool luxo_jr_motion_end_flag_ = false;
bool scene_delay_end_flag_ = false;

int scene_ = 1;

#endif // LUXO_CORE_CONFIG_H_
