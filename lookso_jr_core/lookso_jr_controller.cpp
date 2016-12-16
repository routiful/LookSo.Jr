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

#include "lookso_jr_controller.h"

LookSoJrController::LookSoJrController()
: baudrate_(BAUDRATE),
  protocol_version_(PROTOCOL_VERSION)
{
}

LookSoJrController::~LookSoJrController()
{
  closeDynamixel();
}

bool LookSoJrController::init(void)
{
  portHandler_   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
  packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (portHandler_->openPort())
  {
    // nh.loginfo("Succeeded to open the port!");
  }
  else
  {
    // nh.loginfo("Failed to open the port!");
    // nh.loginfo("Press any key to terminate...");
    // return;
  }

  // Set port baudrate
  if (portHandler_->setBaudRate(baudrate_))
  {
    // nh.loginfo("Succeeded to change the baudrate!");
  }
  else
  {
    // nh.loginfo("Failed to change the baudrate!");
    // nh.loginfo("Press any key to terminate...");
    // return;
  }

  groupSyncWrite_ = new dynamixel::GroupSyncWrite(portHandler_, packetHandler_, ADDR_AX_GOAL_POSITION, LEN_AX_GOAL_POSITION);

  //Enable Dynamixel Torque
  for (int id = 1; id < MOTOR_NUM+1; id++)
  {
    setTorque(id, TORQUE_ENABLE);
  }
}

bool LookSoJrController::setTorque(uint8_t id, bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_AX_TORQUE_ENABLE, onoff, &dxl_error);

  if(dxl_comm_result != COMM_SUCCESS)
      packetHandler_->printTxRxResult(dxl_comm_result);
  else if(dxl_error != 0)
      packetHandler_->printRxPacketError(dxl_error);
}

bool LookSoJrController::setLED(bool onoff)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  for (int id = 1; id < MOTOR_NUM+1; id++)
  {
    dxl_comm_result = packetHandler_->write1ByteTxRx(portHandler_, id, ADDR_AX_LED, onoff, &dxl_error);
    if(dxl_comm_result != COMM_SUCCESS)
        packetHandler_->printTxRxResult(dxl_comm_result);
    else if(dxl_error != 0)
        packetHandler_->printRxPacketError(dxl_error);
  }
}

void LookSoJrController::closeDynamixel(void)
{
  // Disable Dynamixel Torque
  for (int id = 1; id < MOTOR_NUM+1; id++)
  {
    setTorque(id, TORQUE_DISABLE);
  }

  // Close port
  portHandler_->closePort();
}

bool LookSoJrController::readPosition(int8_t id, int *position)
{
  uint8_t dynamixel_error = 0;
  int dynamixel_comm_result = COMM_RX_FAIL;

  int16_t value_16_bit = 0;

  dynamixel_comm_result = packetHandler_->read2ByteTxRx(portHandler_, id, ADDR_AX_PRESENT_POSITION, (uint16_t*)&value_16_bit, &dynamixel_error);

  if (dynamixel_comm_result == COMM_SUCCESS)
  {
    if (dynamixel_error != 0)
    {
      packetHandler_->printRxPacketError(dynamixel_error);
    }

    *position = value_16_bit;
    return true;
  }
  else
  {
    packetHandler_->printTxRxResult(dynamixel_comm_result);
    //ROS_WARN("[ID] %u, Fail to read!, %d", id);
    return false;
  }
}

bool LookSoJrController::positionControl(int goal_position[4])
{
  bool dxl_addparam_result_;
  int8_t dxl_comm_result_;

  for (int id = 1; id < MOTOR_NUM+1; id++)
  {
    dxl_addparam_result_ = groupSyncWrite_->addParam(id, (uint8_t*)&goal_position[id-1]);
    if (dxl_addparam_result_ != true)
    {
      // logerror("[ID:%03d] groupSyncWrite addparam failed", left_wheel_id_);
      return false;
    }
  }

  dxl_comm_result_ = groupSyncWrite_->txPacket();

  if (dxl_comm_result_ != COMM_SUCCESS)
  {
    packetHandler_->printTxRxResult(dxl_comm_result_);
    return false;
  }

  groupSyncWrite_->clearParam();
  return true;
}

int LookSoJrController::convertRadian2Value(float radian)
{
  int value = 0;
  if (radian > 0)
  {
    if (VALUE_OF_MAX_RADIAN_POSITION <= VALUE_OF_0_RADIAN_POSITION)
      return VALUE_OF_MAX_RADIAN_POSITION;

    value = (radian * (VALUE_OF_MAX_RADIAN_POSITION - VALUE_OF_0_RADIAN_POSITION) / MAX_RADIAN)
                + VALUE_OF_0_RADIAN_POSITION;
  }
  else if (radian < 0)
  {
    if (VALUE_OF_MIN_RADIAN_POSITION >= VALUE_OF_0_RADIAN_POSITION)
      return VALUE_OF_MIN_RADIAN_POSITION;

    value = (radian * (VALUE_OF_MIN_RADIAN_POSITION - VALUE_OF_0_RADIAN_POSITION) / MIN_RADIAN)
                + VALUE_OF_0_RADIAN_POSITION;
  }
  else
    value = VALUE_OF_0_RADIAN_POSITION;

  // if (value > VALUE_OF_MAX_RADIAN_POSITION)
  //   return VALUE_OF_MAX_RADIAN_POSITION;
  // else if (value < VALUE_OF_MIN_RADIAN_POSITION)
  //   return VALUE_OF_MIN_RADIAN_POSITION;

  return value;
}

float LookSoJrController::convertValue2Radian(int value)
{
  float radian = 0.0;
  if (value > VALUE_OF_0_RADIAN_POSITION)
  {
    if (MAX_RADIAN <= 0)
      return MAX_RADIAN;

    radian = (float) (value - VALUE_OF_0_RADIAN_POSITION) * MAX_RADIAN
               / (float) (VALUE_OF_MAX_RADIAN_POSITION - VALUE_OF_0_RADIAN_POSITION);
  }
  else if (value < VALUE_OF_0_RADIAN_POSITION)
  {
    if (MIN_RADIAN >= 0)
      return MIN_RADIAN;

    radian = (float) (value - VALUE_OF_0_RADIAN_POSITION) * MIN_RADIAN
               / (float) (VALUE_OF_MIN_RADIAN_POSITION - VALUE_OF_0_RADIAN_POSITION);
  }

//  if (radian > max_radian_)
//    return max_radian_;
//  else if (radian < min_radian_)
//    return min_radian_;

  return radian;
}
