/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "interface/KeyBoard.h"
#include <iostream>

KeyBoard::KeyBoard(){
    userCmd = UserCommand::NONE; // 首先设置用户命令为NONE
    userValue.setZero(); // 设置用户命令值为0

    tcgetattr( fileno( stdin ), &_oldSettings ); // 获取当前标准输入(键盘)的终端设置，保存在_oldSettings中
    _newSettings = _oldSettings; // 将_oldSettings赋值给_newSettings
    _newSettings.c_lflag &= (~ICANON & ~ECHO); // 修改终端设置，关闭规范模式(ICANON)和回显(ECHO)，这样使输入的字符立即可用(无需等待回车键)，且在终端不显示输入的字符
    tcsetattr( fileno( stdin ), TCSANOW, &_newSettings ); // 将修改后的终端设置立即生效

    pthread_create(&_tid, NULL, runKeyBoard, (void*)this); // 创建线程用来读取键盘输入
}

KeyBoard::~KeyBoard(){
    pthread_cancel(_tid); // 取消线程
    pthread_join(_tid, NULL); // 等待线程结束
    tcsetattr( fileno( stdin ), TCSANOW, &_oldSettings ); // 恢复终端设置
}

UserCommand KeyBoard::checkCmd(){
    switch (_c){
    case '1':
        return UserCommand::L2_B; // L2_B代表passive
    case '2':
        return UserCommand::L2_A; // L2_A代表fixedStand
    case '3':
        return UserCommand::L2_X; // L2_X代表freeStand
    case '4':
        return UserCommand::START; // START代表trotting
    case '6':
        return UserCommand::START_CUSTOM; // START_CUSTOM代表自定义的trotting
#ifdef COMPILE_WITH_MOVE_BASE
    case '5':
        return UserCommand::L2_Y;
#endif  // COMPILE_WITH_MOVE_BASE
    case '0':
        return UserCommand::L1_X; // L1_X代表balanceTest
    case '9':
        return UserCommand::L1_A; // L1_A代表swingTest
    case '8':
        return UserCommand::L1_Y; // L1_Y代表stepTest
    case ' ':
        userValue.setZero(); // 空格键，设置用户值为0
        return UserCommand::NONE; // 返回用户命令为NONE
    default:
        return UserCommand::NONE; // 其他情况，返回用户命令为NONE
    }
}

void KeyBoard::changeValue(){
    switch (_c){
    case 'w':case 'W':
        userValue.ly = min<float>(userValue.ly+sensitivityLeft, 1.0); // 如果持续按下w键，ly值逐渐增大，增量为sensitivityLeft=0.05，最大为1
        break;
    case 's':case 'S':
        userValue.ly = max<float>(userValue.ly-sensitivityLeft, -1.0);
        break;
    case 'd':case 'D':
        userValue.lx = min<float>(userValue.lx+sensitivityLeft, 1.0);
        break;
    case 'a':case 'A':
        userValue.lx = max<float>(userValue.lx-sensitivityLeft, -1.0);
        break;

    case 'i':case 'I': // i和k经过我测试好像没啥用处，这里先不管
        userValue.ry = min<float>(userValue.ry+sensitivityRight, 1.0);
        break;
    case 'k':case 'K':
        userValue.ry = max<float>(userValue.ry-sensitivityRight, -1.0);
        break;
    case 'l':case 'L': // j和l是控制机器人旋转的
        userValue.rx = min<float>(userValue.rx+sensitivityRight, 1.0);
        break;
    case 'j':case 'J':
        userValue.rx = max<float>(userValue.rx-sensitivityRight, -1.0);
        break;
    default:
        break;
    }
}

void* KeyBoard::runKeyBoard(void *arg){ // void *类型被用作一种通用指针类型，可以指向任何类型的数据，但在使用时需要进行类型转换
    ((KeyBoard*)arg)->run(NULL); // 这里包含了一个类型转换，将void*类型的arg转换为KeyBoard*类型，然后调用run()函数
    return NULL;
}

void* KeyBoard::run(void *arg){
    while(1){
        FD_ZERO(&set); // 将set清零,set是一个fd_set类型的变量，用来存储文件描述符
        FD_SET( fileno( stdin ), &set ); // 将标准终端输入(键盘)加入到set中，即要用select()函数监视的文件描述符

        // select()函数用来监视文件描述符的变化，如果文件描述符没有变化，select()函数会阻塞，直到文件描述符发生变化，select返回-1表示错误，返回0表示超时，返回正数表示有文件描述符发生变化
        res = select( fileno( stdin )+1, &set, NULL, NULL, NULL);

        if(res > 0){
            ret = read( fileno( stdin ), &_c, 1 );
            userCmd = checkCmd();
            if(userCmd == UserCommand::NONE)
                changeValue();
            _c = '\0';
        }
        usleep(1000);
    }
    return NULL;
}