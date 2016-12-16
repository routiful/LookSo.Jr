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

#ifndef LOOKSO_JR_CONTROLLER_H_
#define LOOKSO_JR_CONTROLLER_H_

#include <DynamixelSDK.h>

// Control table address (XM430-W350-T)
#define ADDR_AX_TORQUE_ENABLE            24
#define ADDR_AX_GOAL_POSITION            30
#define ADDR_AX_PRESENT_POSITION         36
#define ADDR_AX_LED                      25

// Data Byte Length
#define LEN_AX_TORQUE_ENABLE            1
#define LEN_AX_GOAL_POSITION            2
#define LEN_AX_PRESENT_POSITION         2
#define LEN_AX_LED                      1

#define PROTOCOL_VERSION                1.0     // Dynamixel protocol version 2.0

#define BAUDRATE                        1000000 // baurd rate of Dynamixel
#define DEVICENAME                      ""      // no need setting on OpenCR

#define MOTOR_NUM                       4
#define TORQUE_ENABLE                   1       // Value for enabling the torque
#define TORQUE_DISABLE                  0       // Value for disabling the torque

#define VALUE_OF_MAX_RADIAN_POSITION    1024
#define VALUE_OF_MIN_RADIAN_POSITION    0
#define VALUE_OF_0_RADIAN_POSITION      512
#define MAX_RADIAN                      2.6
#define MIN_RADIAN                      -2.6

#define REPEAT                          0

class LookSoJrController
{
 public:
  LookSoJrController();
  ~LookSoJrController();
  bool init(void);
  void closeDynamixel(void);
  bool setTorque(uint8_t id, bool onoff);
  bool setLED(bool onoff);
  bool readPosition(int8_t id, int *position);
  bool positionControl(int goal_position[4]);
  int  convertRadian2Value(float radian);
  float convertValue2Radian(int value);

 private:
  int8_t baudrate_;
  float  protocol_version_;
  dynamixel::PortHandler *portHandler_;
  dynamixel::PacketHandler *packetHandler_;
  dynamixel::GroupSyncWrite * groupSyncWrite_;
};

#endif // LOOKSO_JR_CONTROLLER_H_
