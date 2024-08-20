#include "rl_l2gar/rl_l2r/WirelessProcess.h"

void WirelessProcess::receiveHandle(UNITREE_LEGGED_SDK::LowState *lowState){
    // memcpy是用来从一个内存地址开始，将指定长度的数据块复制到另一个内存地址。第一个参数是目标内存地址，第二个参数是源内存地址，第三个参数是要复制的字节数。
#ifdef ROBOT_TYPE_A1
    memcpy(&_keyData, lowState->wirelessRemote, 40);  // note: available in the unitree_legged_sdk_3.2
#endif
#ifdef ROBOT_TYPE_Go1
    memcpy(&_keyData, &lowState->wirelessRemote[0], 40);
#endif  

    if((int)_keyData.btn.components.start == 1)
    {
        userCmd = UserCommand_rl::RUNNING;
    }
    else if((int)_keyData.btn.components.select == 1)
    {
        userCmd = UserCommand_rl::PASSIVE;
    }
    else if((int)_keyData.btn.components.L1 == 1 && (int)_keyData.btn.components.B == 1)
    {
        userCmd = UserCommand_rl::PAUSE;
    }

    userValue.L2 = killZeroOffset(_keyData.L2, 0.08);  // 如果从手柄上读取的速度命令绝对值小于0.08，则设置为0，这里应该是为了消除
    userValue.lx = killZeroOffset(_keyData.lx, 0.08);
    userValue.ly = killZeroOffset(_keyData.ly, 0.08);
    userValue.rx = killZeroOffset(_keyData.rx, 0.08);
    userValue.ry = killZeroOffset(_keyData.ry, 0.08);
}