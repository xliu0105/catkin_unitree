/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef FSM_H
#define FSM_H

// FSM States
#include "FSM/FSMState.h"
#include "FSM/State_FixedStand.h"
#include "FSM/State_Passive.h"
#include "FSM/State_FreeStand.h"
#include "FSM/State_Trotting.h"
#include "FSM/State_Trotting_Custom.h"
#include "FSM/State_BalanceTest.h"
#include "FSM/State_SwingTest.h"
#include "FSM/State_StepTest.h"
#include "common/enumClass.h"
#include "control/CtrlComponents.h"
#ifdef COMPILE_WITH_MOVE_BASE
    #include "FSM/State_move_base.h"
#endif  // COMPILE_WITH_MOVE_BASE

struct FSMStateList{
    FSMState *invalid;
    State_Passive *passive;
    State_FixedStand *fixedStand;
    State_FreeStand *freeStand;
    State_Trotting *trotting;
    State_Trotting_Custom *trottingCustom; // 自定义的trotting状态
    State_BalanceTest *balanceTest;
    State_SwingTest *swingTest;
    State_StepTest *stepTest;
#ifdef COMPILE_WITH_MOVE_BASE
    State_move_base *moveBase;
#endif  // COMPILE_WITH_MOVE_BASE

    void deletePtr(){
        delete invalid;
        delete passive;
        delete fixedStand;
        delete freeStand;
        delete trotting;
        delete trottingCustom;
        delete balanceTest;
        delete swingTest;
        delete stepTest;
#ifdef COMPILE_WITH_MOVE_BASE
        delete moveBase;
#endif  // COMPILE_WITH_MOVE_BASE
    }
};

class FSM{
public:
    FSM(CtrlComponents *ctrlComp);
    ~FSM();
    void initialize();
    void run();
private:
    FSMState* getNextState(FSMStateName stateName);
    bool checkSafty();
    CtrlComponents *_ctrlComp; // 控制组件类，类中包含了控制命令、状态估计器、IO接口等
    FSMState *_currentState; // FSMState类是一个基类，子类继承于基类，定义了多种运动状态，父类指针指向子类对象，实现多态
    FSMState *_nextState;
    FSMStateName _nextStateName; // 一个enum class，包含了所有可能的状态
    FSMStateList _stateList; // 定义了一个FSMStateList结构体，包含了所有可能的状态
    FSMMode _mode; // 查看FSM模式，被定义为enum class，只有两种模式：NORMAL和CHANGE
    long long _startTime;
    int count;
};


#endif  // FSM_H