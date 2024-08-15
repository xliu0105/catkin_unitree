/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef LOWLEVELCMD_H
#define LOWLEVELCMD_H

#include "common/mathTypes.h"
#include "common/mathTools.h"

struct MotorCmd{ // 电机控制命令结构体，包括控制模式，目标关节角度，目标关节角速度，目标关节力矩，控制器的Kp和Kd
    unsigned int mode; // 控制模式，0为停转，5为开环缓慢转动，10为闭环伺服控制，大部分情况下使用10
    float q;
    float dq;
    float tau;
    float Kp;
    float Kd;

    MotorCmd(){
        mode = 0;
        q = 0;
        dq = 0;
        tau = 0;
        Kp = 0;
        Kd = 0;
    }
};

struct LowlevelCmd{
    MotorCmd motorCmd[12];

    void setQ(Vec12 q){ // 设置目标关节角度，4条腿的12个关节角度一起设置
        for(int i(0); i<12; ++i){
            motorCmd[i].q = q(i);
        }
    }
    void setQ(int legID, Vec3 qi){ // 设置legID腿的关节角度，一次只能设置一个腿的三个关节角度
        motorCmd[legID*3+0].q = qi(0);
        motorCmd[legID*3+1].q = qi(1);
        motorCmd[legID*3+2].q = qi(2);
    }
    void setQd(Vec12 qd){ // 设置目标关节角速度，4条腿的12个关节角速度一起设置
        for(int i(0); i<12; ++i){
            motorCmd[i].dq = qd(i);
        }
    }
    void setQd(int legID, Vec3 qdi){ // 设置legID腿的关节角速度，一次只能设置一个腿的三个关节角速度
        motorCmd[legID*3+0].dq = qdi(0);
        motorCmd[legID*3+1].dq = qdi(1);
        motorCmd[legID*3+2].dq = qdi(2);
    }
    void setTau(Vec12 tau, Vec2 torqueLimit = Vec2(-50, 50)){ // 设置关节力矩，同时设置关节力矩的上下限，第二个参数有默认值
        for(int i(0); i<12; ++i){
            if(std::isnan(tau(i))){
                printf("[ERROR] The setTau function meets Nan\n");
            }
            motorCmd[i].tau = saturation(tau(i), torqueLimit);
        }
    }
    void setZeroDq(int legID){ // 将legID腿的控制器的关节角速度设置为0
        motorCmd[legID*3+0].dq = 0;
        motorCmd[legID*3+1].dq = 0;
        motorCmd[legID*3+2].dq = 0;
    }
    void setZeroDq(){ // 将所有腿的控制器的关节角速度设置为0
        for(int i(0); i<4; ++i){
            setZeroDq(i);
        }
    }
    void setZeroTau(int legID){ // 将legID腿的控制器的关节力矩设置为0
        motorCmd[legID*3+0].tau = 0;
        motorCmd[legID*3+1].tau = 0;
        motorCmd[legID*3+2].tau = 0;
    }
    void setSimStanceGain(int legID){
        motorCmd[legID*3+0].mode = 10;
        motorCmd[legID*3+0].Kp = 180;
        motorCmd[legID*3+0].Kd = 8;
        motorCmd[legID*3+1].mode = 10;
        motorCmd[legID*3+1].Kp = 180;
        motorCmd[legID*3+1].Kd = 8;
        motorCmd[legID*3+2].mode = 10;
        motorCmd[legID*3+2].Kp = 300;
        motorCmd[legID*3+2].Kd = 15;
    }
    void setRealStanceGain(int legID){
        motorCmd[legID*3+0].mode = 10;
        motorCmd[legID*3+0].Kp = 60;
        motorCmd[legID*3+0].Kd = 5;
        motorCmd[legID*3+1].mode = 10;
        motorCmd[legID*3+1].Kp = 40;
        motorCmd[legID*3+1].Kd = 4;
        motorCmd[legID*3+2].mode = 10;
        motorCmd[legID*3+2].Kp = 80;
        motorCmd[legID*3+2].Kd = 7;
    }
    void setZeroGain(int legID){ // 将legID腿的控制器的Kp和Kd都设置为0
        motorCmd[legID*3+0].mode = 10;
        motorCmd[legID*3+0].Kp = 0;
        motorCmd[legID*3+0].Kd = 0;
        motorCmd[legID*3+1].mode = 10;
        motorCmd[legID*3+1].Kp = 0;
        motorCmd[legID*3+1].Kd = 0;
        motorCmd[legID*3+2].mode = 10;
        motorCmd[legID*3+2].Kp = 0;
        motorCmd[legID*3+2].Kd = 0;
    }
    void setZeroGain(){ // 将所有腿的控制器的Kp和Kd都设置为0
        for(int i(0); i<4; ++i){
            setZeroGain(i);
        }
    }
    void setStableGain(int legID){
        motorCmd[legID*3+0].mode = 10;
        motorCmd[legID*3+0].Kp = 0.8;
        motorCmd[legID*3+0].Kd = 0.8;
        motorCmd[legID*3+1].mode = 10;
        motorCmd[legID*3+1].Kp = 0.8;
        motorCmd[legID*3+1].Kd = 0.8;
        motorCmd[legID*3+2].mode = 10;
        motorCmd[legID*3+2].Kp = 0.8;
        motorCmd[legID*3+2].Kd = 0.8;
    }
    void setStableGain(){
        for(int i(0); i<4; ++i){
            setStableGain(i);
        }
    }
    void setSwingGain(int legID){
        motorCmd[legID*3+0].mode = 10;
        motorCmd[legID*3+0].Kp = 3;
        motorCmd[legID*3+0].Kd = 2;
        motorCmd[legID*3+1].mode = 10;
        motorCmd[legID*3+1].Kp = 3;
        motorCmd[legID*3+1].Kd = 2;
        motorCmd[legID*3+2].mode = 10;
        motorCmd[legID*3+2].Kp = 3;
        motorCmd[legID*3+2].Kd = 2;
    }
};

#endif  //LOWLEVELCMD_H