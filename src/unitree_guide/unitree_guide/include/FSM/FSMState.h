/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FSMSTATE_H
#define FSMSTATE_H

#include <string>
#include <iostream>
#include <unistd.h>
#include "control/CtrlComponents.h"
#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "common/enumClass.h"
#include "common/mathTools.h"
#include "common/mathTypes.h"
#include "common/timeMarker.h"
#include "interface/CmdPanel.h"
#include "control/MPC_Controller.h"

// 机器人的各个状态都有相似性，因此可以继承于同一个基类，即这个FSMState类
class FSMState{
public:
    FSMState(CtrlComponents *ctrlComp, FSMStateName stateName, std::string stateNameString); // 构造函数

    virtual void enter() = 0; //切换进入该状态时调用
    virtual void run() = 0; // 保持在当前运行状态，会被循环执行调用
    virtual void exit() = 0; // 退出该状态时调用
    virtual FSMStateName checkChange() {return FSMStateName::INVALID;} // 检查是否需要切换到其他状态

    FSMStateName _stateName;
    std::string _stateNameString;
    MPC_Controller *_mpcCtrl = nullptr; // MPC控制器指针
protected:
    CtrlComponents *_ctrlComp;
    FSMStateName _nextStateName;

    LowlevelCmd *_lowCmd; // 这里的_lowCmd和_lowState是指向CtrlComponents类中的lowCmd和lowState的指针，在构造函数中初始化
    LowlevelState *_lowState;
    UserValue _userValue;
};

#endif  // FSMSTATE_H