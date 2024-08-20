#ifndef RL_REAL_BRIDGE_H
#define RL_REAL_BRIDGE_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include "rl_l2gar/LowState_rl.h"
#include "unitree_legged_msgs/MotorState.h"
#include "unitree_legged_msgs/IMU.h"
#include "unitree_legged_msgs/LowCmd.h"
#include <stdio.h>
#include "CmdBoard.h"
#include "WirelessProcess.h"
#include "unitree_legged_sdk/quadruped.h"
#include <vector>
#include <iostream>
#include <atomic>
#include <csignal>
#include "unitree_legged_sdk/unitree_legged_sdk.h"  // if unitree_A1, use unitree_legged_sdk-3.2, if unitree_Go1, use unitree_legged_sdk-3.8.0

class rl_real_bridge
{
public:
  rl_real_bridge(ros::NodeHandle &nh);
  ~rl_real_bridge(){};
  void Init();
  void receiveState_publish();  // receive state from real robot and publish it to the topic
  void receiveCmd_send_callBack(const unitree_legged_msgs::LowCmd::ConstPtr &lowCmd_msg);  // receive cmd from the topic and send it to the real robot
  void main_func();  // main loop for the bridge
  void Reset();

private:
  UNITREE_LEGGED_SDK::UDP _udp;
  UNITREE_LEGGED_SDK::Safety _safe;
  UNITREE_LEGGED_SDK::LowCmd _lowCmd;
  UNITREE_LEGGED_SDK::LowState _lowState;

  ros::NodeHandle _nh;
  rl_l2gar::LowState_rl _lowState_msg;  // state message
  ros::Publisher _lowState_pub;
  ros::Subscriber _lowCmd_sub;
  CmdBoard *cmdBoard;
  ros::AsyncSpinner *spinner;

  int _motiontime = 0;
  std::vector<int> order_real_lab;
};

#endif  // RL_REAL_BRIDGE_H