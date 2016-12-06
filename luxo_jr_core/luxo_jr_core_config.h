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

/* Authors: Darby Lim, Kim min jae, Kim Tae Min */

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

#endif // LUXO_CORE_CONFIG_H_
