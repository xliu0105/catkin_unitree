#ifndef CMDBOARD_H
#define CMDBOARD_H

#include "message/unitree_joystick.h"
#include "rl_l2gar/common/enum_class.h"
#include <pthread.h>
#include "interface/CmdPanel.h"
#include "unitree_legged_sdk/unitree_legged_sdk.h"


class CmdBoard{
public:
    CmdBoard(){}
    virtual ~CmdBoard(){}
    UserCommand_rl getUserCmd(){return userCmd;} // 返回用户命令
    UserValue getUserValue(){return userValue;} // 返回用户值
    void setPassive(){userCmd = UserCommand_rl::PASSIVE;}
    void setZero(){userValue.setZero();}
    void reset_userCmd(){userCmd = UserCommand_rl::PASSIVE;}
    void Emergency_Protection(){userCmd = UserCommand_rl::PASSIVE;}
    virtual void receiveHandle(UNITREE_LEGGED_SDK::LowState *lowState){};
protected:
    virtual void* run(void *arg){return NULL;}
    UserCommand_rl userCmd = UserCommand_rl::PASSIVE;  // 默认值为None enum类型
    UserValue userValue;
};

#endif