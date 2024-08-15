/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <sched.h>

#include "control/ControlFrame.h"
#include "control/CtrlComponents.h"
#include "Gait/WaveGenerator.h"
#include "control/BalanceCtrl.h"

#ifdef COMPILE_WITH_REAL_ROBOT
#include "interface/IOSDK.h"
#endif // COMPILE_WITH_REAL_ROBOT

#ifdef COMPILE_WITH_ROS
#include "interface/KeyBoard.h"
#include "interface/IOROS.h"
#endif // COMPILE_WITH_ROS

bool running = true;

// over watch the ctrl+c command
void ShutDown(int sig)
{
    std::cout << "stop the controller" << std::endl;
    running = false;
}

void setProcessScheduler() // 设置进程调度器，将进程调度策略设置为SCHED_FIFO的最大优先级
{
    pid_t pid = getpid(); // 获取当前进程的进程标识符 (PID)
    sched_param param; // sched_param是一个结构体，包含与系统调度相关的参数
    param.sched_priority = sched_get_priority_max(SCHED_FIFO); // 获取指定调度策略SCHED_FIFO的最大优先级
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1)
    {
        std::cout << "[ERROR] Function setProcessScheduler failed." << std::endl;
    }
}

int main(int argc, char **argv)
{
    /* set real-time process */
    setProcessScheduler(); // 设计进行调度策略，感觉关系不是很大
    /* set the print format */
    std::cout << std::fixed << std::setprecision(4); // 设置输出格式，不使用科学计数且输出的小数保留三位小数

#ifdef RUN_ROS // 如果在CMakeLists.txt中定义使用SIMULATION，则这里会init；如果定义使用REALROBOT，则这里不会init。两种模式不能同时使用
    ros::init(argc, argv, "unitree_gazebo_servo");
#endif // RUN_ROS

    IOInterface *ioInter; // 定义IOInterface多态指针，指向ROS的IO接口或者SDK的IO接口，用于接收各类传感器数据和发送控制命令
    CtrlPlatform ctrlPlat; // 定义CtrlPlatform枚举类，用于标识控制平台，这个类只包括GAZEBO和REALROBOT

#ifdef COMPILE_WITH_SIMULATION // 如果编译的是仿真代码，即只在GAZEBO中运行
    ioInter = new IOROS();
    ctrlPlat = CtrlPlatform::GAZEBO;
#endif // COMPILE_WITH_SIMULATION

#ifdef COMPILE_WITH_REAL_ROBOT // 如果编译的是真实机器人代码，即在真实机器人中运行
    ioInter = new IOSDK();
    ctrlPlat = CtrlPlatform::REALROBOT;
#endif // COMPILE_WITH_REAL_ROBOT
    // 创建控制组件对象，包含大量与控制相关的类和状态变量，如控制收发接口，输入输出命令，估计器，平衡控制器，控制频率，控制器运行的平台等
    CtrlComponents *ctrlComp = new CtrlComponents(ioInter);
    ctrlComp->ctrlPlatform = ctrlPlat; // 设置ctrlComp对象的控制平台为ctrlPlat
    ctrlComp->dt = 0.002; // run at 500hz
    ctrlComp->dt_backup = 0.002; // 之后换到MPC控制器可能会改变控制周期，这里添加一个备份，用于恢复控制周期
    ctrlComp->running = &running; // 设置ctrlComp对象的running变量为running

#ifdef ROBOT_TYPE_A1 // 如果选择是A1机器人，创建A1机器人模型
    ctrlComp->robotModel = new A1Robot();
#endif
#ifdef ROBOT_TYPE_Go1 // 如果选择是Go1机器人，创建Go1机器人模型
    ctrlComp->robotModel = new Go1Robot();
#endif
    // 这里的步态生成器是WaveGenerator类，用于生成步态参数，包括步态周期、步态相位、步态幅度等
    //------------可选多种步态，取消注释即可使用----------------
    ctrlComp->waveGen = new WaveGenerator(0.45, 0.5, Vec4(0, 0.5, 0.5, 0)); // Trot
    // ctrlComp->waveGen = new WaveGenerator(1.1, 0.75, Vec4(0, 0.25, 0.5, 0.75));  //Crawl, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.6, Vec4(0, 0.5, 0.5, 0));  //Walking Trot, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.35, Vec4(0, 0.5, 0.5, 0));  //Running Trot, only for sim
    // ctrlComp->waveGen = new WaveGenerator(0.4, 0.7, Vec4(0, 0, 0, 0));  //Pronk, only for sim

    ctrlComp->geneObj(); // 创建状态估计器和平衡控制器对象

    ControlFrame ctrlFrame(ctrlComp); // ControlFrame中只包含一个FSM和CtrlComponents类，主要包含run函数，用于调用FSM的run函数

    signal(SIGINT, ShutDown); // 注册ctrl+c信号，当按下ctrl+c时，调用ShutDown函数

    while (running)
    {
        ctrlFrame.run();
    }

    delete ctrlComp;
    return 0;
}
