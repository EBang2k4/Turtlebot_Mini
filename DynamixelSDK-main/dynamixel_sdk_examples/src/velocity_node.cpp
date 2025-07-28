/*******************************************************************************
 * Copyright 2020 ROBOTIS CO., LTD.
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
 *
 * This example is written for DYNAMIXEL X (excluding XL-320) and MX(2.0) series with U2D2.
 * To test this example, follow these commands:
 *
 * Open terminal #1
 * $ roscore
 *
 * Open terminal #2
 * $ rosrun dynamixel_sdk_examples velocity_node
 *
 * Open terminal #3 (run one of the commands below at a time)
 * $ rostopic pub -1 /set_velocity dynamixel_sdk_examples/SetVelocity "{id: 1, velocity: 100}"
 * $ rostopic pub -1 /set_velocity dynamixel_sdk_examples/SetVelocity "{id: 1, velocity: -100}"
 * $ rosservice call /get_velocity "id: 1"
 * $ rostopic pub -1 /set_velocity dynamixel_sdk_examples/SetVelocity "{id: 2, velocity: 100}"
 * $ rostopic pub -1 /set_velocity dynamixel_sdk_examples/SetVelocity "{id: 2, velocity: -100}"
 * $ rosservice call /get_velocity "id: 2"
 *
 * Author: Zerom, Modified for Velocity Control
*******************************************************************************/

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "dynamixel_sdk_examples/GetVelocity.h"
#include "dynamixel_sdk_examples/SetVelocity.h"
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace dynamixel;

// Control table address for X-series
#define ADDR_OPERATING_MODE   11
#define ADDR_TORQUE_ENABLE    64
#define ADDR_GOAL_VELOCITY    104
#define ADDR_PRESENT_VELOCITY 128

// Protocol version
#define PROTOCOL_VERSION      2.0

// Default setting
#define DXL1_ID               1
#define DXL2_ID               2
#define BAUDRATE              57600
#define DEVICE_NAME           "/dev/ttyACM0"
#define VELOCITY_CONTROL_MODE 1

PortHandler *portHandler;
PacketHandler *packetHandler;

bool getPresentVelocityCallback(
  dynamixel_sdk_examples::GetVelocity::Request & req,
  dynamixel_sdk_examples::GetVelocity::Response & res)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Velocity Value of X series is 4 byte data
  int32_t velocity = 0;

  // Read Present Velocity (length: 4 bytes)
  dxl_comm_result = packetHandler->read4ByteTxRx(
    portHandler, (uint8_t)req.id, ADDR_PRESENT_VELOCITY, (uint32_t *)&velocity, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("getVelocity : [ID:%d] -> [VELOCITY:%d]", req.id, velocity);
    res.velocity = velocity;
    return true;
  } else {
    ROS_ERROR("Failed to get velocity! Result: %d", dxl_comm_result);
    return false;
  }
}

void setVelocityCallback(const dynamixel_sdk_examples::SetVelocity::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Velocity Value of X series is 4 byte data
  int32_t velocity = msg->velocity; // Use int32_t for velocity (signed)

  // Write Goal Velocity (length: 4 bytes)
  dxl_comm_result = packetHandler->write4ByteTxRx(
    portHandler, (uint8_t)msg->id, ADDR_GOAL_VELOCITY, velocity, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("setVelocity : [ID:%d] [VELOCITY:%d]", msg->id, msg->velocity);
  } else {
    ROS_ERROR("Failed to set velocity! Result: %d", dxl_comm_result);
  }
}

int main(int argc, char ** argv)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  portHandler = PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  // Open port
  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  // Set baudrate
  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }

  // Set Operating Mode to Velocity Control for DXL1
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to set Velocity Control Mode for Dynamixel ID %d", DXL1_ID);
    return -1;
  }

  // Enable Torque for DXL1
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL1_ID);
    return -1;
  }
  uint8_t torque_status;
  packetHandler->read1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, &torque_status, &dxl_error);
  ROS_INFO("Torque status DXL1: %d", torque_status);

  // Set Operating Mode to Velocity Control for DXL2
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL2_ID, ADDR_OPERATING_MODE, VELOCITY_CONTROL_MODE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to set Velocity Control Mode for Dynamixel ID %d", DXL2_ID);
    return -1;
  }

  // Enable Torque for DXL2
  dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 1, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL2_ID);
    return -1;
  }
  packetHandler->read1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, &torque_status, &dxl_error);
  ROS_INFO("Torque status DXL2: %d", torque_status);

  // Initialize ROS node
  ros::init(argc, argv, "velocity_node");
  ros::NodeHandle nh;

  // Create service for getting velocity
  ros::ServiceServer get_velocity_srv = nh.advertiseService("/get_velocity", getPresentVelocityCallback);

  // Create subscriber for setting velocity
  ros::Subscriber set_velocity_sub = nh.subscribe("/set_velocity", 10, setVelocityCallback);

  // Spin to process callbacks
  ros::spin();

  // Stop motors by setting velocity to 0
  packetHandler->write4ByteTxRx(portHandler, DXL1_ID, ADDR_GOAL_VELOCITY, 0, &dxl_error);
  packetHandler->write4ByteTxRx(portHandler, DXL2_ID, ADDR_GOAL_VELOCITY, 0, &dxl_error);

  // Disable torque
  packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);
  packetHandler->write1ByteTxRx(portHandler, DXL2_ID, ADDR_TORQUE_ENABLE, 0, &dxl_error);

  // Close port
  portHandler->closePort();
  return 0;
}
