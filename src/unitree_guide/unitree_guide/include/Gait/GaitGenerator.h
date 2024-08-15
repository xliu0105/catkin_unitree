/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef GAITGENERATOR_H
#define GAITGENERATOR_H

#include "Gait/WaveGenerator.h"
#include "Gait/FeetEndCal.h"

#ifdef COMPILE_DEBUG
#include <common/PyPlot.h>
#endif  // COMPILE_DEBUG

/*cycloid gait*/
class GaitGenerator{ // 这个类的作用是计算摆线的轨迹，在FSM的_currentState->run()中调用
public:
    GaitGenerator(CtrlComponents *ctrlComp);
    ~GaitGenerator();
    void setGait(Vec2 vxyGoalGlobal, float dYawGoal, float gaitHeight); // 设置世界坐标系下的期望机身平移速度，角速度以及抬腿高度，直接赋值给类内的成员变量
    void run(Vec34 &feetPos, Vec34 &feetVel); // 控制器每轮都会调用这个函数，
    Vec3 getFootPos(int i); // 获取对应足端当前时刻的目标位置
    Vec3 getFootVel(int i); // 获取对应足端当前时刻的目标速度
    void restart();
private:
    float cycloidXYPosition(float startXY, float endXY, float phase); // 计算摆线的x、y坐标
    float cycloidXYVelocity(float startXY, float endXY, float phase); // 计算摆线的x、y速度
    float cycloidZPosition(float startZ, float height, float phase); // 计算摆线的z坐标
    float cycloidZVelocity(float height, float phase); // 计算摆线的z速度

    WaveGenerator *_waveG; // 
    Estimator *_est; // 状态估计器
    FeetEndCal *_feetCal; // 落脚点计算类
    QuadrupedRobot *_robModel;
    LowlevelState *_state;
    float _gaitHeight; // 期望抬腿高度
    Vec2 _vxyGoal; // 世界坐标系下期望机身平移速度
    float _dYawGoal; // 期望z轴角速度
    Vec4 *_phase, _phasePast;
    VecInt4 *_contact;
    Vec34 _startP, _endP, _idealP, _pastP;
    bool _firstRun; // 是否是第一次运行，一个marker

#ifdef COMPILE_DEBUG
    PyPlot _testGaitPlot;
#endif  // COMPILE_DEBUG

};

#endif  // GAITGENERATOR_H