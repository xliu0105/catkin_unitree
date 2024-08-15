/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef LOWLEVELSTATE_HPP
#define LOWLEVELSTATE_HPP

#include <iostream>
#include "common/mathTypes.h"
#include "common/mathTools.h"
#include "interface/CmdPanel.h"
#include "common/enumClass.h"

struct MotorState // 电机状态结构体，包括控制模式，电机角度，电机角速度，电机角加速度，电机估计力矩
{
	unsigned int mode; // 控制模式，0为停转，5为开环缓慢转动，10为闭环伺服控制，大部分情况下为10
    float q;
    float dq;
    float ddq;
    float tauEst;

    MotorState(){
        q = 0;
        dq = 0;
        ddq = 0;
        tauEst = 0;
    }
};

struct IMU
{
    float quaternion[4];    //姿态， w, x, y, z
    float gyroscope[3]; // 陀螺仪，三个方向的角速度
    float accelerometer[3]; // 加速度，三维空间的加速度

    IMU(){ // 初始化IMU数据，所有数据都设置为0
        for(int i = 0; i < 3; i++){
            quaternion[i] = 0;
            gyroscope[i] = 0;
            accelerometer[i] = 0;
        }
        quaternion[3] = 0;
    }

    RotMat getRotMat(){ // 根据当前的四元数计算出旋转矩阵并返回，使用了mathTools.h中的函数
        Quat quat;
        quat << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return quatToRotMat(quat);
    }

    Vec3 getAcc(){ // 返回加速度
        Vec3 acc;
        acc << accelerometer[0], accelerometer[1], accelerometer[2];
        return acc;
    }

    Vec3 getGyro(){ // 返回陀螺仪数据，即角速度
        Vec3 gyro;
        gyro << gyroscope[0], gyroscope[1], gyroscope[2];
        return gyro;
    }

    Quat getQuat(){ // 返回四元数
        Quat q;
        q << quaternion[0], quaternion[1], quaternion[2], quaternion[3];
        return q;
    }
};

struct LowlevelState
{
    IMU imu; // 定义IMU数据
    MotorState motorState[12]; // 定义MotorState数组，包含12个元素，是12个电机的状态和控制命令
    UserCommand userCmd; // 一个enum类型的变量，表示用户的命令，包括停止，站立，行走等状态
    UserValue userValue; // 一个结构体，包含用户的输入值

    Vec34 getQ(){ // 返回12个电机的角度，形成一个3*4的矩阵
        Vec34 qLegs;
        for(int i(0); i < 4; ++i){
            qLegs.col(i)(0) = motorState[3*i    ].q;
            qLegs.col(i)(1) = motorState[3*i + 1].q;
            qLegs.col(i)(2) = motorState[3*i + 2].q;
        }
        return qLegs;
    }

    Vec34 getQd(){ // 返回12个电机的角速度，形成一个3*4的矩阵
        Vec34 qdLegs;
        for(int i(0); i < 4; ++i){
            qdLegs.col(i)(0) = motorState[3*i    ].dq;
            qdLegs.col(i)(1) = motorState[3*i + 1].dq;
            qdLegs.col(i)(2) = motorState[3*i + 2].dq;
        }
        return qdLegs;
    }

    RotMat getRotMat(){ // 返回imu获得姿态的旋转矩阵
        return imu.getRotMat();
    }

    Vec3 getAcc(){ // 返回imu获得的加速度
        // std::cout << "imu.getAcc(): " << imu.getAcc() << std::endl;
        return imu.getAcc();
    }

    Vec3 getGyro(){ // 返回imu获得的角速度
        return imu.getGyro();
    }

    Vec3 getAccGlobal(){ // 返回全局坐标系下的加速度
        return getRotMat() * getAcc();
    }

    Vec3 getGyroGlobal(){ // 返回全局坐标系下的角速度，是一个向量
        return getRotMat() * getGyro();
    }

    double getYaw(){ // 返回z轴偏航角，使用了mathTools.h中的函数
        return rotMatToRPY(getRotMat())(2);
    }

    double getDYaw(){ // 返回z轴角速度，一个标量
        return getGyroGlobal()(2);
    }

    void setQ(Vec12 q){ // 设置12个电机的角度
        for(int i(0); i<12; ++i){
            motorState[i].q = q(i);
        }
    }
};

#endif  //LOWLEVELSTATE_HPP