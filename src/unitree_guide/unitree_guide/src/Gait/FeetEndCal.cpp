/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "Gait/FeetEndCal.h"

FeetEndCal::FeetEndCal(CtrlComponents *ctrlComp)
           : _est(ctrlComp->estimator), _lowState(ctrlComp->lowState),
             _robModel(ctrlComp->robotModel){ // 初始化状态估计，底层状态和机器人模型
    _Tstance  = ctrlComp->waveGen->getTstance(); // 获取触地阶段的时间
    _Tswing   = ctrlComp->waveGen->getTswing(); // 获取摆腿阶段的时间

    _kx = 0.005; // 计算落脚点的反馈系数，参考宇树教材的公式9.8和9.10
    _ky = 0.005;
    _kyaw = 0.005;

    Vec34 feetPosBody = _robModel->getFeetPosIdeal(); // 获取所有足端中性落脚点在机身坐标系下的坐标
    for(int i(0); i<4; ++i){
        _feetRadius(i)    = sqrt( pow(feetPosBody(0, i), 2) + pow(feetPosBody(1, i), 2) ); // 计算机器人绕自身中心点旋转的半径，即公式中的R
        _feetInitAngle(i) = atan2(feetPosBody(1, i), feetPosBody(0, i)); // 计算大腿关节与机身中心连线到机身坐标系x轴的夹角
    }
}

FeetEndCal::~FeetEndCal(){

}

Vec3 FeetEndCal::calFootPos(int legID, Vec2 vxyGoalGlobal, float dYawGoal, float phase){
    _bodyVelGlobal = _est->getVelocity(); // 获取当前机身在世界坐标系下的速度
    _bodyWGlobal = _lowState->getGyroGlobal(); // 获取当前机身在世界坐标系下的角速度向量，是3维向量

    // 以下的计算完全参考宇树教材的公式9.5、9.8、9.9、9.10
    _nextStep(0) = _bodyVelGlobal(0)*(1-phase)*_Tswing + _bodyVelGlobal(0)*_Tstance/2 + _kx*(_bodyVelGlobal(0) - vxyGoalGlobal(0));
    _nextStep(1) = _bodyVelGlobal(1)*(1-phase)*_Tswing + _bodyVelGlobal(1)*_Tstance/2 + _ky*(_bodyVelGlobal(1) - vxyGoalGlobal(1));
    _nextStep(2) = 0;

    _yaw = _lowState->getYaw(); // 获取当前z轴偏航角
    _dYaw = _lowState->getDYaw(); // 获取当前z轴偏航角速度，double类型标量
    _nextYaw = _dYaw*(1-phase)*_Tswing + _dYaw*_Tstance/2 + _kyaw*(dYawGoal - _dYaw);

    _nextStep(0) += _feetRadius(legID) * cos(_yaw + _feetInitAngle(legID) + _nextYaw);
    _nextStep(1) += _feetRadius(legID) * sin(_yaw + _feetInitAngle(legID) + _nextYaw);

    _footPos = _est->getPosition() + _nextStep;
    _footPos(2) = 0.0; // 落脚点没有z坐标，因此设为0

    return _footPos;
}