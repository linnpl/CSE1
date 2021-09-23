// Copyright 2020 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*******************************************************************************
 * This example is written for DYNAMIXEL X(excluding XL-320) and MX(2.0) series with U2D2.
 * For other series, please refer to the product eManual and modify the Control Table addresses and other definitions.
 * To test this example, please follow the commands below.
 *
 * Open terminal #1
 * $ roscore
 *
 * Open terminal #2
 * $ rosrun dynamixel_sdk_examples read_write_node
 *
 * Open terminal #3 (run one of below commands at a time)
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 0}"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 1, position: 1000}"
 * $ rosservice call /get_position "id: 1"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 2, position: 0}"
 * $ rostopic pub -1 /set_position dynamixel_sdk_examples/SetPosition "{id: 2, position: 1000}"
 * $ rosservice call /get_position "id: 2"
 *
 * Author: Zerom
*******************************************************************************/

#include <ros/ros.h>

#include "std_msgs/String.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include "mx106_driver/mx106.h"

using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    24
#define ADDR_GOAL_POSITION    30
#define ADDR_PRESENT_POSITION 36

// Protocol version
#define PROTOCOL_VERSION      1.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID               1               // DXL1 ID
#define DXL2_ID               2               // DXL2 ID
#define BAUDRATE              57600           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

PortHandler * portHandler;
PacketHandler * packetHandler;
int numberOfServos;

/**
 * @brief Constructor
 */
Mx106::Mx106()
{
}

/**
 * @brief get current Position of specific servo
 * @param id uint8_t id of the servo with the position you want to read
 * @retval position -4096 is negative 180deg + 4095 is positive 180deg
 */
int16_t Mx106::getPresentPosition(uint8_t id)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
  int16_t position = 0;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  dxl_comm_result = packetHandler->read2ByteTxRx(
    portHandler, id, ADDR_PRESENT_POSITION, (uint16_t *)&position, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", id, position);
    return position;
  } else {
    ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
  }
}

void Mx106::getPresentPositions(uint8_t ids[], int16_t positions[])
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(int16_t) for the Position Value.
  int16_t position = 0;

  // Read Present Position (length : 4 bytes) and Convert uint32 -> int32
  // When reading 2 byte data from AX / MX(1.0), use read2ByteTxRx() instead.
  for (int i = 0; i < numberOfServos; i++)
  {
    dxl_comm_result = packetHandler->read2ByteTxRx(
      portHandler, ids[i], ADDR_PRESENT_POSITION, (uint16_t *)&position, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", ids[i], position);
      positions[i] = position;
    } else {
      ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
    }
  }
}

/**
 * @brief set position of single servo
 * @param position position you want the servo set to
 * @param id id of the servo you want to set the position of
 */
void Mx106::setPosition(int16_t position, uint8_t id)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  //uint16_t position = (unsigned int16_t)position; // Convert int32 -> uint32

  // Write Goal Position (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  dxl_comm_result = packetHandler->write2ByteTxRx(
    portHandler, id, ADDR_GOAL_POSITION, position, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", id, position);
  } else {
    ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
  }
}

/**
 * @brief set position of single servo
 * @param positions positions you want the servo set to
 * @param ids ids of the servo you want to set the position of
 */
void Mx106::setPositions(int16_t positions[], uint8_t ids[])
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
  //uint16_t position = (unsigned int16_t)position; // Convert int32 -> uint32

  // Write Goal Position (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  for (int i = 0; i < numberOfServos; i++)
  {
    dxl_comm_result = packetHandler->write2ByteTxRx(
      portHandler, ids[i], ADDR_GOAL_POSITION, positions[i], &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", ids[i], positions[i]);
    } else {
      ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
    }
  }
}

/**
 * @brief initializes all dynamixel mx106 servos
 * @param ids uint8_t array of all servos IDs.
 * @retval 0 Success
 * @retval -1 Failure
 */
int Mx106::initServos(uint8_t ids[], int Servos){  
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  numberOfServos = Servos;

  portHandler = PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }

  for (int i = 0; i < (numberOfServos); i++)
  {
    dxl_comm_result = packetHandler->write1ByteTxRx(
      portHandler, ids[i], ADDR_TORQUE_ENABLE, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      ROS_ERROR("Failed to enable torque for Dynamixel ID 1  %d", ids[i]);
      return -1;
    }
  }
}

/*
bool getPresentPositionCallback(
  dynamixel_sdk_examples::GetPosition::Request & req,
  dynamixel_sdk_examples::GetPosition::Response & res)
{
  res.position = getPresentPosition(req.id);
}


void setPositionCallback(const dynamixel_sdk_examples::SetPosition::ConstPtr & msg)
{
  int32_t pos = &msg.position;
  setPosition(pos, msg.id)
}
*/

/*
int main(int argc, char ** argv)
{
  int8_t servoIds[] = {4,5,6};
  initServos(servoIds);
  int16_t counter = 0;

  ros::init(argc, argv, "new_node");
  ros::NodeHandle nh;
  ros::Rate rate(1);

  while (ros::ok()){
    
    rate.sleep();
    setPosition(counter, 4);
    setPosition(counter, 5);
    setPosition(counter, 6);
    counter = counter + 50;
    ROS_INFO("%d ", counter);
  }
  ros::spin();

  portHandler->closePort();
  return 0;
}
*/
