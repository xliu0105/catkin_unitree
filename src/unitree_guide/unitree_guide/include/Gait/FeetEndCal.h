/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FEETENDCAL_H
#define FEETENDCAL_H

#include "control/CtrlComponents.h"
#include "message/LowlevelState.h"

class FeetEndCal{ // 这个类的作用是计算当前机器人状态下目标落脚点的坐标，会在GaitGenerator类中调用
public:
    FeetEndCal(CtrlComponents *ctrlComp);
    ~FeetEndCal();
    // 要计算落脚点的坐标，可以调用calFootPos函数，参数分别为：
    // legID: 要计算落脚点的腿的编号
    // vxyGoalGlobal: 机器人的目标速度，Vec2类型，包含x和y方向的速度
    // dYawGoal: 机器人的目标角速度
    // phase: 该腿当前的相位
    Vec3 calFootPos(int legID, Vec2 vxyGoalGlobal, float dYawGoal, float phase);
private:
    LowlevelState *_lowState;
    Estimator *_est;
    QuadrupedRobot *_robModel;

    Vec3 _nextStep, _footPos;
    Vec3 _bodyVelGlobal;        // linear velocity
    Vec3 _bodyAccGlobal;        // linear accelerator
    Vec3 _bodyWGlobal;          // angular velocity

    Vec4 _feetRadius, _feetInitAngle;
    float _yaw, _dYaw, _nextYaw;

    float _Tstance, _Tswing;
    float _kx, _ky, _kyaw; // 反馈系数，参考宇树教材的公式9.8和9.10
};

#endif  // FEETENDCAL_H