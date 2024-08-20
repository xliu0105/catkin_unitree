#include "rl_l2gar/rl_l2r/rl_real_bridge.h"
#include <chrono>
#include <thread>

static std::atomic<bool> keepRunning{true};  // NOTE: 静态全局变量，仅在本文件可见，用于控制程序是否继续运行

static void CtrlC_Handler(int signum)  // NOTE: 静态全局函数，仅在本文件可见，用于处理Ctrl+C信号
{  
  std::cout << "Interrupt signal (" << signum << ") received." << std::endl;
  keepRunning = false;
}

#ifdef ROBOT_TYPE_A1
rl_real_bridge::rl_real_bridge(ros::NodeHandle &nh): _nh(nh), _safe(UNITREE_LEGGED_SDK::LeggedType::A1), _udp(UNITREE_LEGGED_SDK::LOWLEVEL)
{
  this->Init();
}
#endif

#ifdef ROBOT_TYPE_Go1
// TODO: 为Go1机器人添加构造函数
#endif

void rl_real_bridge::Init()
{
  ROS_INFO("The control bridge for real robot");
  _udp.InitCmdData(_lowCmd);
  cmdBoard = new WirelessProcess();  // WirelessHandle类继承于CmdBoard类，重写了receiveHandle方法，专门用于处理实体机器人的遥控器数据
  _lowState_pub = _nh.advertise<rl_l2gar::LowState_rl>("/realRobot/LowState", 1);
  _lowCmd_sub = _nh.subscribe<unitree_legged_msgs::LowCmd>("/realRobot/LowCmd", 1, &rl_real_bridge::receiveCmd_send_callBack, this);
  spinner = new ros::AsyncSpinner(5);  // 10 threads
  spinner->start();
  signal(SIGINT, &CtrlC_Handler);  // 注册Ctrl+C信号处理函数

  {  // 这里的{}是用来限制using namespace的作用域，防止污染命名空间
    using namespace UNITREE_LEGGED_SDK;
    order_real_lab = {FL_0,FR_0,RL_0,RR_0,FL_1,FR_1,RL_1,RR_1,FL_2,FR_2,RL_2,RR_2};  // 将真实机器人的12个关节按照isaac lab的顺序排列
  }
}

void rl_real_bridge::Reset()
{
  _motiontime = 0;
  cmdBoard->setZero();
  cmdBoard->reset_userCmd();
}

void rl_real_bridge::receiveState_publish()  // 之后打算用unitree_legged_sdk中提供的Loop函数，因此这里面就不写循环了
{
    float check_data = 0;  // 用来检查数据是否可靠，如果_udp没有正常链接或没有接收到数据，则所有的数据都会是0
    _udp.Recv();
    _udp.GetRecv(_lowState);
    for(int i(0); i < 12; i++)
    {
      _lowState_msg.motorState[i].q = _lowState.motorState[order_real_lab[i]].q;
      _lowState_msg.motorState[i].dq = _lowState.motorState[order_real_lab[i]].dq;
      _lowState_msg.motorState[i].ddq = _lowState.motorState[order_real_lab[i]].ddq;
      _lowState_msg.motorState[i].tauEst = _lowState.motorState[order_real_lab[i]].tauEst;
      _lowState_msg.motorState[i].mode = _lowState.motorState[order_real_lab[i]].mode;
      check_data = check_data + _lowState_msg.motorState[i].q + _lowState_msg.motorState[i].dq + _lowState_msg.motorState[i].ddq + 
                    _lowState_msg.motorState[i].tauEst + _lowState_msg.motorState[i].mode;
    }

    for(int i(0); i < 3; i++)
    {
      _lowState_msg.imu.quaternion[i] = _lowState.imu.quaternion[i];
      _lowState_msg.imu.gyroscope[i] = _lowState.imu.gyroscope[i];
      _lowState_msg.imu.accelerometer[i] = _lowState.imu.accelerometer[i];
      check_data = check_data + _lowState_msg.imu.quaternion[i] + _lowState_msg.imu.gyroscope[i] + _lowState_msg.imu.accelerometer[i];
    }
    _lowState_msg.imu.quaternion[3] = _lowState.imu.quaternion[3];
    check_data += _lowState_msg.imu.quaternion[3];

    cmdBoard->receiveHandle(&_lowState);

    UserValue usr_value_buf = cmdBoard->getUserValue();
    _lowState_msg.userValue.lx = usr_value_buf.lx;
    _lowState_msg.userValue.ly = usr_value_buf.ly;
    _lowState_msg.userValue.rx = usr_value_buf.rx;
    _lowState_msg.userValue.ry = usr_value_buf.ry;
    _lowState_msg.userValue.L2 = usr_value_buf.L2;

    check_data = check_data + usr_value_buf.lx + usr_value_buf.ly + usr_value_buf.rx + usr_value_buf.ry + usr_value_buf.L2;

    _lowState_msg.userCmd = static_cast<int>(cmdBoard->getUserCmd());  // 将枚举类型转换为整数类型

    check_data += _lowState_msg.userCmd;

    if(check_data == 0)
    {
      ROS_WARN("No data received from the real robot. Please check the connection. No data will be published.");
      sleep(1);
    }
    else
    {
      _lowState_pub.publish(_lowState_msg);
    }
}

void rl_real_bridge::receiveCmd_send_callBack(const unitree_legged_msgs::LowCmd::ConstPtr &lowCmd_msg)
{
  for(int i(0); i < 12; i++)
  {
    _lowCmd.motorCmd[order_real_lab[i]].mode = lowCmd_msg->motorCmd[i].mode;
    _lowCmd.motorCmd[order_real_lab[i]].q = lowCmd_msg->motorCmd[i].q;
    _lowCmd.motorCmd[order_real_lab[i]].dq = lowCmd_msg->motorCmd[i].dq;
    _lowCmd.motorCmd[order_real_lab[i]].Kp = lowCmd_msg->motorCmd[i].Kp;
    _lowCmd.motorCmd[order_real_lab[i]].Kd = lowCmd_msg->motorCmd[i].Kd;
    _lowCmd.motorCmd[order_real_lab[i]].tau = lowCmd_msg->motorCmd[i].tau;
  }
  if(_motiontime < 100)
  {
    _motiontime++;
  }
  if(_motiontime > 10)
  {
    _safe.PositionLimit(_lowCmd);
    _safe.PowerProtect(_lowCmd, _lowState, 2);
      // You can uncomment it for position protection
    // safe.PositionProtect(_lowCmd, _lowState);
  }

  _udp.SetSend(_lowCmd);
  _udp.Send();
}

void rl_real_bridge::main_func()
{
  std::cout << "Communication level is set to LOW-level." << std::endl
            << "WARNING: Make sure the robot is hung up." << std::endl
            << "Press Enter to continue..." << std::endl;
  std::cin.ignore();
  sleep(0.5);

  UNITREE_LEGGED_SDK::LoopFunc loop_udp_recv_pub("udp_recv_pub", 0.001, 3, boost::bind(&rl_real_bridge::receiveState_publish, this));
  loop_udp_recv_pub.start();
  while(keepRunning)
  {
    sleep(1);
  }
}

