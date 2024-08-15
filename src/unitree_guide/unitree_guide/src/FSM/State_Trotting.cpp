/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/State_Trotting.h"
#include <iomanip>

State_Trotting::State_Trotting(CtrlComponents *ctrlComp)
             :FSMState(ctrlComp, FSMStateName::TROTTING, "trotting"), 
              _est(ctrlComp->estimator), _phase(ctrlComp->phase), 
              _contact(ctrlComp->contact), _robModel(ctrlComp->robotModel), 
              _balCtrl(ctrlComp->balCtrl){
    _gait = new GaitGenerator(ctrlComp);

    _gaitHeight = 0.08;

#ifdef ROBOT_TYPE_Go1
    _Kpp = Vec3(70, 70, 70).asDiagonal();
    _Kdp = Vec3(10, 10, 10).asDiagonal();
    _kpw = 780; 
    _Kdw = Vec3(70, 70, 70).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

#ifdef ROBOT_TYPE_A1
    _Kpp = Vec3(20, 20, 100).asDiagonal();
    _Kdp = Vec3(20, 20, 20).asDiagonal();
    _kpw = 400;
    _Kdw = Vec3(50, 50, 50).asDiagonal();
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

    _vxLim = _robModel->getRobVelLimitX();
    _vyLim = _robModel->getRobVelLimitY();
    _wyawLim = _robModel->getRobVelLimitYaw();

}

State_Trotting::~State_Trotting(){
    delete _gait;
}

void State_Trotting::enter(){ // 切换进入该状态时调用
    _pcd = _est->getPosition(); // 令机器人的位置等于当前估计器估计的位置
    _pcd(2) = -_robModel->getFeetPosIdeal()(2, 0); // 这个函数返回的是各个足端中性落脚点在机身坐标系下的坐标，因此这里的_z前面有个负号(-)
    _vCmdBody.setZero(); // 令机身在机身坐标系下的目标速度为0
    _yawCmd = _lowState->getYaw(); // 令目标偏航角等于当前估计的偏航角
    _Rd = rotz(_yawCmd); // 设置机身目标姿态的旋转矩阵为当前姿态的旋转矩阵
    _wCmdGlobal.setZero(); // 令世界坐标系下，目标角速度为0
    _ctrlComp->ioInter->zeroCmdPanel(); // 命令信号清零
    _gait->restart(); // 步态重新从起点开始
}

void State_Trotting::exit(){
    _ctrlComp->ioInter->zeroCmdPanel(); // 将所有命令信号清零
    _ctrlComp->setAllSwing(); // 将所有足端设置为摆动状态，为什么要这样做？
}

FSMStateName State_Trotting::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::TROTTING;
    }
}

void State_Trotting::run(){
    _posBody = _est->getPosition(); // 获取当前机身在世界坐标系下的位置，由估计器估计得到
    _velBody = _est->getVelocity(); // 获取当前机身在世界坐标系下的速度，由估计器估计得到
    _posFeet2BGlobal = _est->getPosFeet2BGlobal(); // 获取所有足端在世界坐标系下相对于机身中心的位置，由估计器估计得到
    _posFeetGlobal = _est->getFeetPos(); // 获取所有足端在世界坐标系下的位置，由估计器估计得到
    _velFeetGlobal = _est->getFeetVel(); // 获取所有足端在世界坐标系下的速度，由估计器估计得到
    _B2G_RotMat = _lowState->getRotMat(); // 获取当前机身坐标系到世界坐标系的旋转矩阵，直接从imu中读取并计算得到
    _G2B_RotMat = _B2G_RotMat.transpose(); // 获取当前世界坐标系到机身坐标系的旋转矩阵
    _yaw = _lowState->getYaw(); // 获取当前偏航角，直接从imu中读取并计算得到
    _dYaw = _lowState->getDYaw(); // 获取当前偏航角速度，直接从imu中读取并计算得到

    _userValue = _lowState->userValue; // 获取用户通过键盘或手柄发送的控制命令

    getUserCmd(); // 获取并处理用户的控制命令
    calcCmd(); // 计算机器人在世界坐标系下的目标速度和角速度，以及下一时刻的目标位置和姿态

    _gait->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight); // _gait是一个GaitGenerator类的对象，用来计算摆线的轨迹
    _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

    calcTau(); // 计算各个足端的足端力，支撑腿的足端力由平衡控制器_balCtrl计算，摆动腿的足端力是由教材公式5.48计算得到的修正力
    calcQQd(); // 根据足端的目标位置和速度，计算得到该腿各个关节的角度和角速度

    if(checkStepOrNot()){ // 使用checkStepOrNot()函数来判断是否要踏步，即当运动速度、位置误差、速度误差、偏航角速度等超过一定阈值时，返回true
        _ctrlComp->setStartWave(); // 调用这个函数可以切换为踏步
    }else{
        _ctrlComp->setAllStance(); // 调用这个函数可以切换为四足站立
    }

    _lowCmd->setTau(_tau); // 将计算得到的关节力矩发送给低级控制器
    _lowCmd->setQ(vec34ToVec12(_qGoal)); // 将计算得到的关节角度发送给低级控制器
    _lowCmd->setQd(vec34ToVec12(_qdGoal)); // 将计算得到的关节角速度发送给低级控制器

    for(int i(0); i<4; ++i){ // 对于摆动腿来说，其目的是跟踪轨迹，因此其应接近于位置控制，对摆动腿设置setSwingGain，将刚度设为较大的数值
        if((*_contact)(i) == 0){
            _lowCmd->setSwingGain(i);
        }else{
            _lowCmd->setStableGain(i); // 支撑腿进行力控制，对支撑腿设置setStableGain，相对较小的刚度
        }
    }
}

bool State_Trotting::checkStepOrNot(){
    if( (fabs(_vCmdBody(0)) > 0.03) ||
        (fabs(_vCmdBody(1)) > 0.03) ||
        (fabs(_posError(0)) > 0.08) ||
        (fabs(_posError(1)) > 0.08) ||
        (fabs(_velError(0)) > 0.05) ||
        (fabs(_velError(1)) > 0.05) ||
        (fabs(_dYawCmd) > 0.20) ){ // 如果机器人的速度、位置误差、速度误差、偏航角速度等超过一定阈值，则返回true，机器人要切换到踏步状态
        return true;
    }else{
        return false;
    }
}

void State_Trotting::setHighCmd(double vx, double vy, double wz){
    _vCmdBody(0) = vx;
    _vCmdBody(1) = vy;
    _vCmdBody(2) = 0; 
    _dYawCmd = wz;
}

void State_Trotting::getUserCmd(){
    /* Movement */
    _vCmdBody(0) =  invNormalize(_userValue.ly, _vxLim(0), _vxLim(1)); // invNormalize反归一化函数，将输入值映射到_vxLim(0)和_vxLim(1)之间
    _vCmdBody(1) = -invNormalize(_userValue.lx, _vyLim(0), _vyLim(1));
    _vCmdBody(2) = 0; // z轴方向的速度始终为0

    /* Turning */
    _dYawCmd = -invNormalize(_userValue.rx, _wyawLim(0), _wyawLim(1));
    _dYawCmd = 0.9*_dYawCmdPast + (1-0.9) * _dYawCmd; // 这里可以看出，角速度的更新是缓慢的，类似于低通滤波，这样可以减小机器人的抖动
    _dYawCmdPast = _dYawCmd; // 记录上一次的偏航角速度
}

void State_Trotting::calcCmd(){
    /* Movement */
    _vCmdGlobal = _B2G_RotMat * _vCmdBody; // 机身坐标系下的速度转换到世界坐标系下

    _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-0.2, _velBody(0)+0.2)); // 限制_vCmdGlobal(0)的范围在_velBody(0)-0.2和_velBody(0)+0.2之间
    _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1)-0.2, _velBody(1)+0.2)); // 限制_vCmdGlobal(1)的范围在当前速度减0.2和当前速度加0.2之间

    _pcd(0) = saturation(_pcd(0) + _vCmdGlobal(0) * _ctrlComp->dt, Vec2(_posBody(0) - 0.05, _posBody(0) + 0.05)); // 同上
    _pcd(1) = saturation(_pcd(1) + _vCmdGlobal(1) * _ctrlComp->dt, Vec2(_posBody(1) - 0.05, _posBody(1) + 0.05));

    _vCmdGlobal(2) = 0; // z轴方向的速度始终为0

    /* Turning */
    _yawCmd = _yawCmd + _dYawCmd * _ctrlComp->dt; // 更新偏航角

    _Rd = rotz(_yawCmd); // 更新目标姿态的旋转矩阵
    _wCmdGlobal(2) = _dYawCmd; // 更新目标角速度向量，只有z轴方向有值
}

void State_Trotting::calcTau(){
    _posError = _pcd - _posBody; //计算机身在世界坐标系下的位置误差
    _velError = _vCmdGlobal - _velBody; // 计算机身在世界坐标系下的速度误差

    _ddPcd = _Kpp * _posError + _Kdp * _velError; // 计算机身在世界坐标系下的目标线加速度，这里的目标线加速度是由位置误差和速度误差计算得到的
    // 计算机身在世界坐标系下的目标角加速度，是由姿态和角速度误差计算得到的
    _dWbd  = _kpw*rotMatToExp(_Rd*_G2B_RotMat) + _Kdw * (_wCmdGlobal - _lowState->getGyroGlobal());

    _ddPcd(0) = saturation(_ddPcd(0), Vec2(-3, 3)); // 限制目标线加速度的范围
    _ddPcd(1) = saturation(_ddPcd(1), Vec2(-3, 3));
    _ddPcd(2) = saturation(_ddPcd(2), Vec2(-5, 5));
    
    _dWbd(0) = saturation(_dWbd(0), Vec2(-40, 40));
    _dWbd(1) = saturation(_dWbd(1), Vec2(-40, 40));
    _dWbd(2) = saturation(_dWbd(2), Vec2(-10, 10));

    _forceFeetGlobal = - _balCtrl->calF(_ddPcd, _dWbd, _B2G_RotMat, _posFeet2BGlobal, *_contact); // 计算支撑腿的足端力，调用了平衡控制器_balCtrl的calF函数

    for(int i(0); i<4; ++i){ // 计算摆动腿的足端力
        if((*_contact)(i) == 0){
            _forceFeetGlobal.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
        } // 这里并没有腿的动力学前馈，只有基于位置误差和速度误差的PD计算力矩控制，因此如果要让足抬腿高度符合要求，必须再控制关节的角度和角速度。
          // 宇树的电机允许同时发送关节力矩，角度和角速度命令，可以看宇树的教材P12页2.7.6节
    }
    _forceFeetBody = _G2B_RotMat * _forceFeetGlobal; // 将足端力转换到机身坐标系下
    _q = vec34ToVec12(_lowState->getQ()); // 获取当前关节角度
    _tau = _robModel->getTau(_q, _forceFeetBody); // 计算关节力矩，其实就是Jocabian矩阵的转置乘以足端力
}

void State_Trotting::calcQQd(){

    Vec34 _posFeet2B;
    _posFeet2B = _robModel->getFeet2BPositions(*_lowState,FrameType::BODY);
    
    for(int i(0); i<4; ++i){
        _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody);
        _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody); 
        // _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody - _B2G_RotMat * (skew(_lowState->getGyro()) * _posFeet2B.col(i)) );  //  c.f formula (6.12) 
    }
    
    _qGoal = vec12ToVec34(_robModel->getQ(_posFeet2BGoal, FrameType::BODY));
    _qdGoal = vec12ToVec34(_robModel->getQd(_posFeet2B, _velFeet2BGoal, FrameType::BODY));
}

