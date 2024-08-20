/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_REAL_ROBOT

#include "interface/WirelessHandle.h"
#include "common/mathTools.h"
#include <string.h>
#include <stdio.h>

WirelessHandle::WirelessHandle(){
}

void WirelessHandle::receiveHandle(UNITREE_LEGGED_SDK::LowState *lowState){
#ifdef ROBOT_TYPE_A1
    memcpy(&_keyData, lowState->wirelessRemote, 40);  // note: available in the unitree_legged_sdk_3.2
#endif
#ifdef ROBOT_TYPE_Go1
    memcpy(&_keyData, &lowState->wirelessRemote[0], 40);
#endif  
    if(((int)_keyData.btn.components.L2 == 1) &&  // 如果同时按下手柄上的L2和B键，则是passive模式
       ((int)_keyData.btn.components.B  == 1)){
        userCmd = UserCommand::L2_B;
    }
    else if(((int)_keyData.btn.components.L2 == 1) &&  // 如果同时按下手柄上的L2和A键，则是fixedStand模式
            ((int)_keyData.btn.components.A  == 1)){
        userCmd = UserCommand::L2_A;
    }
    else if(((int)_keyData.btn.components.L2 == 1) &&  // 如果同时按下手柄上的L2和X键，则是freeStand模式
            ((int)_keyData.btn.components.X  == 1)){
        userCmd = UserCommand::L2_X;
    }

#ifdef COMPILE_WITH_MOVE_BASE
    else if(((int)_keyData.btn.components.L2 == 1) && 
            ((int)_keyData.btn.components.Y  == 1)){
        userCmd = UserCommand::L2_Y;
    }
#endif  // COMPILE_WITH_MOVE_BASE

    else if(((int)_keyData.btn.components.L1 == 1) && 
            ((int)_keyData.btn.components.X  == 1)){
        userCmd = UserCommand::L1_X;
    }
    else if(((int)_keyData.btn.components.L1 == 1) && 
            ((int)_keyData.btn.components.A  == 1)){
        userCmd = UserCommand::L1_A;
    }
    else if(((int)_keyData.btn.components.L1 == 1) && 
            ((int)_keyData.btn.components.Y  == 1)){
        userCmd = UserCommand::L1_Y;
    }
    else if((int)_keyData.btn.components.start == 1){  // 如果按下手柄上的start键，则是trotting模式
        userCmd = UserCommand::START;
    }

    userValue.L2 = killZeroOffset(_keyData.L2, 0.08);  // 如果从手柄上读取的速度命令绝对值小于0.08，则设置为0，这里应该是为了消除
    userValue.lx = killZeroOffset(_keyData.lx, 0.08);
    userValue.ly = killZeroOffset(_keyData.ly, 0.08);
    userValue.rx = killZeroOffset(_keyData.rx, 0.08);
    userValue.ry = killZeroOffset(_keyData.ry, 0.08);
}


#endif  // COMPILE_WITH_REAL_ROBOT
