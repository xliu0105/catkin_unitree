#ifndef WIRELESSPROCESS_H
#define WIRELESSPROCESS_H

#include "message/unitree_joystick.h"
#include "CmdBoard.h"
#include "unitree_legged_sdk/comm.h"
#include "common/mathTools.h"
#include <string.h>
#include <stdio.h>

class WirelessProcess : public CmdBoard{
public:
    WirelessProcess(){};
    ~WirelessProcess(){};
    void receiveHandle(UNITREE_LEGGED_SDK::LowState *lowState);
private:
    xRockerBtnDataStruct _keyData;
};

#endif