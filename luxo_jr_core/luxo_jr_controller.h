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

/* Authors: Darby Lim, Kim Min Jae, Kim Tae Min */

#ifndef LUXO_JR_CONTROLLER_H_
#define LUXO_JR_CONTROLLER_H_

#include <DynamixelSDK.h>

// Control table address (XM430-W350-T)
#define ADDR_AX_TORQUE_ENABLE            24
#define ADDR_XM_GOAL_POSITION            30
#define ADDR_XM_PRESENT_POSITION         36

// Data Byte Length
#define LEN_XM_TORQUE_ENABLE            1
#define LEN_XM_GOAL_POSITION            2
#define LEN_XM_PRESENT_POSITION         2

#define PROTOCOL_VERSION                1.0     // Dynamixel protocol version 2.0

#define BAUDRATE                        1000000 // baurd rate of Dynamixel
#define DEVICENAME                      ""      // no need setting on OpenCR

#define MOTOR_NUM                       1
#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

#define DEGREE2RADIAN                   (PI / 180.0)
#define RADIAN2DEGREE                   (180.0 / PI)

#define JOINT_STEP                      0.005   //radian

class LuxoJrController
{
 public:
  LuxoJrController();
  ~LuxoJrController();
  bool init(void);
  void closeDynamixel(void);
  bool setTorque(uint8_t id, bool onoff);
  bool readPosition(int8_t id, int *position);
  bool positionControl(float goal_position[4]);

 private:
  int8_t baudrate_;
  float  protocol_version_;
  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;
  dynamixel::GroupSyncWrite * groupSyncWrite_;
};

#endif // LUXO_JR_CONTROLLER_H_
