/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef WAVEGENERATOR_H
#define WAVEGENERATOR_H

#include "common/mathTypes.h"
#include "common/timeMarker.h"
#include "common/enumClass.h"
#include <unistd.h>

#ifdef COMPILE_DEBUG
#include "common/PyPlot.h"
#endif  // COMPILE_DEBUG

/*generate linear wave, [0, 1]*/
class WaveGenerator{ // 这个类的主要作用是根据时间生成四条腿的步态和相位
public:
    // period为步态周期，stancePhaseRatio代表支撑相占一个周期的比例(触地系数)，bias是一个4维vector，代表了四条腿的相位偏差，具体见教材
    WaveGenerator(double period, double stancePhaseRatio, Vec4 bias);
    ~WaveGenerator();
    
    // 计算当前时刻四个足端的相位phaseResult和接触状态contactResult，1代表足端触地，0代表足端离地。status代表当前机器人的步态状态，
    // 有三种状态：STANCE_ALL代表四条腿都在支撑，SWING_ALL代表四条腿都在腾空，WAVE_ALL代表四条腿都在运动
    // calcContactPhase中会调用calcWave函数，根据当前时刻的状态计算出四条腿的相位和接触状态
    void calcContactPhase(Vec4 &phaseResult, VecInt4 &contactResult, WaveStatus status);
    float getTstance(); // 返回足端的触地时长
    float getTswing(); // 返回足端的腾空时长
    float getT(); // 返回步态周期P
    float getStanceRatio(); // 返回支撑相占一个周期的比例
    Vec4 getBias(); // 返回四条腿的相位偏差
private:
    void calcWave(Vec4 &phase, VecInt4 &contact, WaveStatus status); // 计算当前时刻四个足端位于步态周期的哪个位置，以及四个足端的接触状态

    double _period; // 步态周期
    double _stRatio; // 支撑相占一个周期的比例(触地系数)
    Vec4 _bias; // 四条腿的相位偏差

    Vec4 _normalT;                   // [0, 1)
    Vec4 _phase, _phasePast; // _phase代表当前处于触地(或腾空)的相位，不是占步态周期的相位，而是占触地阶段或腾空阶段的相位
    VecInt4 _contact, _contactPast; // 当前的接触状态和过去的接触状态
    VecInt4 _switchStatus;          // 1: switching, 0: do not switch
    WaveStatus _statusPast; // 过去的步态状态

    double _passT;                   // unit: second
    long long _startT;    // unit: us，为WaveGenerator类被创建的系统时间
#ifdef COMPILE_DEBUG
    PyPlot _testPlot;
#endif  // COMPILE_DEBUG

};

#endif  // WAVEGENERATOR_H