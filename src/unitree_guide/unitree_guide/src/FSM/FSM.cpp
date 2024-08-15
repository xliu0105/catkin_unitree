/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "FSM/FSM.h"
#include <iostream>

FSM::FSM(CtrlComponents *ctrlComp)
    :_ctrlComp(ctrlComp){

    _stateList.invalid = nullptr;
    _stateList.passive = new State_Passive(_ctrlComp);
    _stateList.fixedStand = new State_FixedStand(_ctrlComp);
    _stateList.freeStand = new State_FreeStand(_ctrlComp);
    _stateList.trotting = new State_Trotting(_ctrlComp);
    _stateList.trottingCustom = new State_Trotting_Custom(_ctrlComp); // 初始化自定义的trotting状态
    _stateList.balanceTest = new State_BalanceTest(_ctrlComp);
    _stateList.swingTest = new State_SwingTest(_ctrlComp);
    _stateList.stepTest = new State_StepTest(_ctrlComp);
#ifdef COMPILE_WITH_MOVE_BASE
    _stateList.moveBase = new State_move_base(_ctrlComp);
#endif  // COMPILE_WITH_MOVE_BASE
    initialize();
}

FSM::~FSM(){
    _stateList.deletePtr();
}

void FSM::initialize(){ // 初始化FSM，将所有之后在run循环中需要用到的变量初始化
    _currentState = _stateList.passive; // 构造FSM时，将当前状态设为passive状态
    _currentState -> enter(); // 调用当前状态的enter函数
    _nextState = _currentState; // 将下一个状态设为当前状态，即passive状态
    _mode = FSMMode::NORMAL; // 将FSM的模式设为NORMAL
}

void FSM::run(){
    _startTime = getSystemTime(); // 获取当前系统时间
    _ctrlComp->sendRecv(); // 控制命令收发一次，在这一部分可以接收键盘或手柄指令，进行FSM模式的切换
    _ctrlComp->runWaveGen(); // 计算步态参数，主要是计算当前时刻的接触状态和相位
    _ctrlComp->estimator->run(); // 状态估计器迭代一次
    if(!checkSafty()){ // 进行安全检测
        _ctrlComp->ioInter->setPassive(); // 若不符合安全条件，则设置各关节进入阻尼状态以保护机器人
    }

    // 查看FSM定义的mode，被定义为enum class格式，只有两种模式：NORMAL和CHANGE
    if(_mode == FSMMode::NORMAL){
        _currentState->run();
        _nextStateName = _currentState->checkChange();
        if(_nextStateName != _currentState->_stateName){
            _mode = FSMMode::CHANGE;
            _nextState = getNextState(_nextStateName);
            std::cout << "Switched from " << _currentState->_stateNameString
                      << " to " << _nextState->_stateNameString << std::endl;
        }
    }
    else if(_mode == FSMMode::CHANGE){
        _currentState->exit();
        _currentState = _nextState;
        _currentState->enter();
        _mode = FSMMode::NORMAL;
        _currentState->run();
    }

    // 在FSM::run()函数一开始通过getSystemTime()获取了当前的系统时间，然后执行完以上所有的内容后，到达这里的absoluteWait()函数，
    // 执行该函数也会首先得到一个执行完所有逻辑后的系统时间，会和_ctrlComp->dt指定的时间相减(转化为微秒)。如果结果为正，则表示超时，
    // 终端会报一个程序警告；如果结果为负，责表示程序跑的太快了，可调用usleep()函数来延时。
    // if(_currentState->_stateName != FSMStateName::CUSTOM_TROTTING)
    // {
    absoluteWait(_startTime, (long long)(_ctrlComp->dt * 1000000));
    // }
    // else
    // {
    //     absoluteWait(_startTime, (long long)(_currentState->_mpcCtrl->_dt * 1000000));
    // }
}

FSMState* FSM::getNextState(FSMStateName stateName){
    switch (stateName)
    {
    case FSMStateName::INVALID:
        return _stateList.invalid;
        break;
    case FSMStateName::PASSIVE:
        return _stateList.passive;
        break;
    case FSMStateName::FIXEDSTAND:
        return _stateList.fixedStand;
        break;
    case FSMStateName::FREESTAND:
        return _stateList.freeStand;
        break;
    case FSMStateName::TROTTING:
        return _stateList.trotting;
        break;
    case FSMStateName::CUSTOM_TROTTING: // 如果按6键，切换返回自定义的trotting状态
        return _stateList.trottingCustom;
        break;
    case FSMStateName::BALANCETEST:
        return _stateList.balanceTest;
        break;
    case FSMStateName::SWINGTEST:
        return _stateList.swingTest;
        break;
    case FSMStateName::STEPTEST:
        return _stateList.stepTest;
        break;
#ifdef COMPILE_WITH_MOVE_BASE
    case FSMStateName::MOVE_BASE:
        return _stateList.moveBase;
        break;
#endif  // COMPILE_WITH_MOVE_BASE
    default:
        return _stateList.invalid;
        break;
    }
}

bool FSM::checkSafty(){
    // The angle with z axis less than 60 degree
    // 这个保护策略的意思是，如果机器人的z轴与世界坐标系的z轴的夹角小于60度，则返回false，否则返回true
    // 注意，这个保护策略默认添加于整个工程的FSM运行状态下，如果要验证一些极端的运动，如前空翻，后空翻，拜年等，需要注意此策略的影响。
    if(_ctrlComp->lowState->getRotMat()(2,2) < 0.5 ){
        return false;
    }else{
        return true;
    }
}