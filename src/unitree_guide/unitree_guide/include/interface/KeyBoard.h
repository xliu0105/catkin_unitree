/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef KEYBOARD_H
#define KEYBOARD_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include "interface/CmdPanel.h"
#include "common/mathTools.h"

class KeyBoard : public CmdPanel{
public:
    KeyBoard();
    ~KeyBoard();
private:
    static void* runKeyBoard(void *arg);
    void* run(void *arg);
    UserCommand checkCmd();
    void changeValue();

    pthread_t _tid;
    float sensitivityLeft = 0.05; // 在持续按下w键或s键时，增量值
    float sensitivityRight = 0.05;
    struct termios _oldSettings, _newSettings;
    fd_set set;
    int res;
    int ret;
    char _c; // 读取键盘输入的字符存储在_c中
};

#endif  // KEYBOARD_H