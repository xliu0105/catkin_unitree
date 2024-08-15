/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "Gait/GaitGenerator.h"

GaitGenerator::GaitGenerator(CtrlComponents *ctrlComp)
              : _waveG(ctrlComp->waveGen), _est(ctrlComp->estimator), 
                _phase(ctrlComp->phase), _contact(ctrlComp->contact), 
                _robModel(ctrlComp->robotModel), _state(ctrlComp->lowState){
    _feetCal = new FeetEndCal(ctrlComp); // 初始化落脚点计算器
    _firstRun = true;
}

GaitGenerator::~GaitGenerator(){
}

void GaitGenerator::setGait(Vec2 vxyGoalGlobal, float dYawGoal, float gaitHeight){ // 设置相关的参数
    _vxyGoal = vxyGoalGlobal;
    _dYawGoal = dYawGoal;
    _gaitHeight = gaitHeight;
}

void GaitGenerator::restart(){ // 重新开始，将_firstRun置为true，同时期望机身平移速度设为0
    _firstRun = true;
    _vxyGoal.setZero();
}

void GaitGenerator::run(Vec34 &feetPos, Vec34 &feetVel){
    if(_firstRun){ // 如果是第一次运行
        _startP = _est->getFeetPos(); // 获取所有足端在世界坐标系下的位置
        _firstRun = false; // 将_firstRun置为false
    }

    for(int i(0); i<4; ++i){
        if((*_contact)(i) == 1){ // 如果这个足端触地
            if((*_phase)(i) <= 0.5){ // 如果当前触地足端的相位小于0.5，这里为什么要判断小于0.5呢，可能是因为在触地的前半段，通过est估计的足端位置是不准确的？
                _startP.col(i) = _est->getFootPos(i); // 获取当前足端在世界坐标系下的位置
            }
            feetPos.col(i) = _startP.col(i); // 将当前足端的位置赋值给feetPos
            feetVel.col(i).setZero(); // 由于是触地状态，所以速度为0
        }
        else{ // 如果这个足端腾空
            _endP.col(i) = _feetCal->calFootPos(i, _vxyGoal, _dYawGoal, (*_phase)(i)); // 计算目标落脚点的坐标

            feetPos.col(i) = getFootPos(i); // 获取摆动足端当前的目标位置
            feetVel.col(i) = getFootVel(i); // 获取摆动足端当前的目标速度
        }
    }
    _pastP = feetPos; // 将当前的足端位置赋值给_pastP作为历史足端位置
    _phasePast = *_phase;  // 将当前的相位赋值给_phasePast作为历史相位
}

// 
Vec3 GaitGenerator::getFootPos(int i){
    Vec3 footPos;

    footPos(0) = cycloidXYPosition(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footPos(1) = cycloidXYPosition(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    footPos(2) =  cycloidZPosition(_startP.col(i)(2), _gaitHeight, (*_phase)(i));
    
    return footPos;
}

Vec3 GaitGenerator::getFootVel(int i){
    Vec3 footVel;

    footVel(0) = cycloidXYVelocity(_startP.col(i)(0), _endP.col(i)(0), (*_phase)(i));
    footVel(1) = cycloidXYVelocity(_startP.col(i)(1), _endP.col(i)(1), (*_phase)(i));
    footVel(2) =  cycloidZVelocity(_gaitHeight, (*_phase)(i));

    return footVel;
}

// 计算摆线的x、y坐标
float GaitGenerator::cycloidXYPosition(float start, float end, float phase){
    float phasePI = 2 * M_PI * phase;
    return (end - start)*(phasePI - sin(phasePI))/(2*M_PI) + start; // 完全参照宇树教材的公式9.14
}

// 计算摆线的x、y速度
float GaitGenerator::cycloidXYVelocity(float start, float end, float phase){
    float phasePI = 2 * M_PI * phase;
    return (end - start)*(1 - cos(phasePI)) / _waveG->getTswing(); // 完全参照宇树教材的公式9.15
}

// 计算摆线的z坐标
float GaitGenerator::cycloidZPosition(float start, float h, float phase){
    float phasePI = 2 * M_PI * phase;
    return h*(1 - cos(phasePI))/2 + start; // 完全参照宇树教材的公式9.14
}

// 计算摆线的z速度
float GaitGenerator::cycloidZVelocity(float h, float phase){
    float phasePI = 2 * M_PI * phase;
    return h*M_PI * sin(phasePI) / _waveG->getTswing(); // 完全参照宇树教材的公式9.15
}