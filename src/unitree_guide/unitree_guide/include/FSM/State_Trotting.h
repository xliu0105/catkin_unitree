/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef TROTTING_H
#define TROTTING_H

#include "FSM/FSMState.h"
#include "Gait/GaitGenerator.h"
#include "control/BalanceCtrl.h"

class State_Trotting : public FSMState{
public:
    State_Trotting(CtrlComponents *ctrlComp);
    ~State_Trotting();
    void enter();
    void run();
    void exit();
    virtual FSMStateName checkChange();
    void setHighCmd(double vx, double vy, double wz);
private:
    void calcTau();
    void calcQQd();
    void calcCmd();
    virtual void getUserCmd();
    void calcBalanceKp();
    bool checkStepOrNot();

    GaitGenerator *_gait;
    Estimator *_est; // 估计器，指向CtrlComponents类中的估计器
    QuadrupedRobot *_robModel; // 机器人模型，包含机器人的各种参数和正逆运动学，指向CtrlComponents类中的robotModel
    BalanceCtrl *_balCtrl; // 平衡控制器，指向CtrlComponents类中的balCtrl

    // Rob State
    Vec3  _posBody, _velBody; // 当前机身在世界坐标系下的位置和速度
    double _yaw, _dYaw; // 当前机身在世界坐标系下的偏航角和偏航角速度
    Vec34 _posFeetGlobal, _velFeetGlobal; // 当前足端在世界坐标系下的位置坐标和速度
    Vec34 _posFeet2BGlobal; // 当前足端在世界坐标系下相对于机身中心的位置坐标
    RotMat _B2G_RotMat, _G2B_RotMat; // 当前机身坐标系到世界坐标系的旋转矩阵，当前世界坐标系到机身坐标系的旋转矩阵
    Vec12 _q; // 各个关节的角度

    // Robot command
    Vec3 _pcd; // 机身在世界坐标系下的目标位置
    Vec3 _vCmdGlobal, _vCmdBody; // 机身在世界坐标系下的目标速度，机身在机身坐标系下的目标速度
    double _yawCmd, _dYawCmd; // _yawCmd是在世界坐标系下的目标偏航角，_dYawCmd是在世界坐标系下的目标偏航角速度
    double _dYawCmdPast; // 上一次的目标偏航角速度(z轴角速度)
    Vec3 _wCmdGlobal; // 机器人在世界坐标系下的目标角速度向量
    Vec34 _posFeetGlobalGoal, _velFeetGlobalGoal; // 足端在世界坐标系下的目标位置坐标和目标速度
    Vec34 _posFeet2BGoal, _velFeet2BGoal; // 足端在机身坐标系下相对于机身中心的目标位置坐标和目标速度
    RotMat _Rd; // 机身目标姿态的旋转矩阵
    Vec3 _ddPcd, _dWbd; // 机身在世界坐标系下的目标线加速度和角加速度
    Vec34 _forceFeetGlobal, _forceFeetBody; // 足端在世界坐标系下的目标足端力，足端在机身坐标系下的目标足端力
    Vec34 _qGoal, _qdGoal; // 各个关节的目标角度和目标角速度
    Vec12 _tau; // 各个关节的前馈力矩

    // Control Parameters
    double _gaitHeight; // 摆动腿轨迹的抬腿高度
    Vec3 _posError, _velError; // 当前时刻机身在世界坐标系下的位置和速度误差
    Mat3 _Kpp, _Kdp, _Kdw; // 用于机身平衡控制器
    double _kpw; // 用于机身平衡控制器
    Mat3 _KpSwing, _KdSwing; // 摆动足端修正力的系数
    Vec2 _vxLim, _vyLim, _wyawLim; // 机身坐标系下x,y方向速度和z轴角速度的限制
    Vec4 *_phase; // 机器人的步态相位，指向CtrlComponents类中的phase
    VecInt4 *_contact; // 机器人的四个腿是否接触地面，指向CtrlComponents类中的contact

    // Calculate average value
    AvgCov *_avg_posError = new AvgCov(3, "_posError", true, 1000, 1000, 1);
    AvgCov *_avg_angError = new AvgCov(3, "_angError", true, 1000, 1000, 1000);
};

#endif  // TROTTING_H