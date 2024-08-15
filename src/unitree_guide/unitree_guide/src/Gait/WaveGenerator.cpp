/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "Gait/WaveGenerator.h"
#include <iostream>
#include <sys/time.h>
#include <math.h>

WaveGenerator::WaveGenerator(double period, double stancePhaseRatio, Vec4 bias)
    : _period(period), _stRatio(stancePhaseRatio), _bias(bias)
{

    if ((_stRatio >= 1) || (_stRatio <= 0)) // 判断触地系数是否在(0, 1)之间
    {
        std::cout << "[ERROR] The stancePhaseRatio of WaveGenerator should between (0, 1)" << std::endl;
        exit(-1);
    }

    for (int i(0); i < bias.rows(); ++i)
    {
        if ((bias(i) > 1) || (bias(i) < 0)) // 判断相位偏差是否在[0, 1]之间
        {
            std::cout << "[ERROR] The bias of WaveGenerator should between [0, 1]" << std::endl;
            exit(-1);
        }
    }

    _startT = getSystemTime(); // 获取这个类被创建的系统时间
    _contactPast.setZero(); // 初始化过去的触地状态
    _phasePast << 0.5, 0.5, 0.5, 0.5; // 初始化过去的_phase相位
    _statusPast = WaveStatus::SWING_ALL; // 初始化过去的步态状态
}

WaveGenerator::~WaveGenerator()
{
}

// 计算当前时刻四个足端的相位phaseResult和接触状态contactResult，并且如果当前时刻的步态状态和过去的步态状态不一样，会进行一些处理
void WaveGenerator::calcContactPhase(Vec4 &phaseResult, VecInt4 &contactResult, WaveStatus status)
{

    calcWave(_phase, _contact, status); // 先计算一次当前时刻当前status的四个足的相位和接触状态

    if (status != _statusPast) // 如果当前时刻的步态状态和过去的步态状态不一样
    {
        if (_switchStatus.sum() == 0) // 如果所有的switchStatus都为0
        {
            _switchStatus.setOnes(); // 将switchStatus全部置为1
        }
        calcWave(_phasePast, _contactPast, _statusPast); // 计算过去时刻的四个足的相位和接触状态
        // two special case
        if ((status == WaveStatus::STANCE_ALL) && (_statusPast == WaveStatus::SWING_ALL)) // 如果当前时刻是STANCE_ALL，过去时刻是SWING_ALL
        {
            // SWING_ALL代表四条腿都在腾空全都是0，要切换到STANCE_ALL，四条腿都要触地，所以过去时刻的接触状态要置为1才符合逻辑
            _contactPast.setOnes();
        }
        else if ((status == WaveStatus::SWING_ALL) && (_statusPast == WaveStatus::STANCE_ALL)) // 如果当前时刻是SWING_ALL，过去时刻是STANCE_ALL
        {
            // STANCE_ALL代表四条腿都在支撑全都是1，要切换到SWING_ALL，四条腿都要离地，所以过去时刻的接触状态要置为0才符合逻辑
            _contactPast.setZero();
        }
    }

    if (_switchStatus.sum() != 0) // 如果switchStatus不全为0
    {
        for (int i(0); i < 4; ++i)
        {
            if (_contact(i) == _contactPast(i)) // 如果当前时刻的接触状态和过去时刻的接触状态一样
            {
                _switchStatus(i) = 0; // 将switchStatus置为0
            }
            else
            {
                _contact(i) = _contactPast(i); // 如果当前时刻的接触状态和过去时刻的接触状态不一样，将当前时刻的接触状态置为过去时刻的接触状态
                _phase(i) = _phasePast(i); // 将当前时刻的相位置为过去时刻的相位
            }
        }
        if (_switchStatus.sum() == 0)
        {
            _statusPast = status; // 如果switchStatus全为0，将过去时刻的步态状态置为当前时刻的步态状态
        }
    }

    phaseResult = _phase; // 将当前时刻的相位传给phaseResult
    contactResult = _contact; // 将当前时刻的接触状态传给contactResult
}

float WaveGenerator::getTstance() // 获取步态的支撑时长
{
    return _period * _stRatio;
}

float WaveGenerator::getTswing() // 获取步态的腾空时长
{
    return _period * (1 - _stRatio);
}

float WaveGenerator::getT() // 获取步态周期
{
    return _period;
}

float WaveGenerator::getStanceRatio() // 获取支撑相占一个周期的比例
{
    return _stRatio;
}

Vec4 WaveGenerator::getBias() // 获取四条腿的相位偏差
{
    return _bias;
}

void WaveGenerator::calcWave(Vec4 &phase, VecInt4 &contact, WaveStatus status)
{
    if (status == WaveStatus::WAVE_ALL) // 判断当前步态状态
    {
        _passT = (double)(getSystemTime() - _startT) * 1e-6; // 获取这个类被创建后到现在的时间，单位为秒
        for (int i(0); i < 4; ++i)
        {
            _normalT(i) = fmod(_passT + _period - _period * _bias(i), _period) / _period; // 根据以下的内容来计算当前处于步态周期的哪个位置，传给phase
            if (_normalT(i) < _stRatio) // 根据触地系数来判断当前时刻的接触状态
            {
                contact(i) = 1;
                phase(i) = _normalT(i) / _stRatio; // _phase代表当前处于触地(或腾空)的相位，不是占步态周期的相位，而是占触地阶段或腾空阶段的相位
            }
            else
            {
                contact(i) = 0;
                phase(i) = (_normalT(i) - _stRatio) / (1 - _stRatio);
            }
        }
    }
    else if (status == WaveStatus::SWING_ALL)
    {
        contact.setZero();
        phase << 0.5, 0.5, 0.5, 0.5;
    }
    else if (status == WaveStatus::STANCE_ALL)
    {
        contact.setOnes();
        phase << 0.5, 0.5, 0.5, 0.5;
    }
}