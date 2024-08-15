/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef UNITREEROBOT_H
#define UNITREEROBOT_H

#include "common/unitreeLeg.h"
#include "message/LowlevelState.h"

class QuadrupedRobot{
public:
    QuadrupedRobot(){};
    ~QuadrupedRobot(){}

    Vec3 getX(LowlevelState &state);
    Vec34 getVecXP(LowlevelState &state);

    // Inverse Kinematics(Body/Hip Frame)
    // 逆运动学，获取当前机器人全部12个关节角度，feetPosition每一列代表足端的位置，frame代表feetPosition参考坐标系，只能为FrameType::BODY或FrameType::HIP
    Vec12 getQ(const Vec34 &feetPosition, FrameType frame);
    // 逆向微分运动学，获取当前机器人全部12个关节角速度，feetPosition每一列代表足端的位置，feetVelocity每一列代表足端的速度，frame代表feetPosition参考坐标系
    Vec12 getQd(const Vec34 &feetPosition, const Vec34 &feetVelocity, FrameType frame);
    // 机器人静力学计算，获取当前机器人全部12个关节力矩，q代表当前机器人全部12个关节角度，feetForce为四个足端的对外作用力
    Vec12 getTau(const Vec12 &q, const Vec34 feetForce);

    // Forward Kinematics
    Vec3 getFootPosition(LowlevelState &state, int id, FrameType frame); // 正向运动学，获取当前机器人第id条腿在frame坐标系下的足端位置，Frame只可以是FrameType::BODY或FrameType::HIP
    Vec3 getFootVelocity(LowlevelState &state, int id); // 正向微分运动学，获取当前机器人第id条腿的足端速度
    Vec34 getFeet2BPositions(LowlevelState &state, FrameType frame); // 获取所有足端相对于机身中心的位置向量，frame代表参考坐标系，可以是FrameType::BODY或FrameType::HIP或FrameType::WORLD
    Vec34 getFeet2BVelocities(LowlevelState &state, FrameType frame); // 获取所有足端相对于机身中心的速度向量，frame代表参考坐标系，可以是FrameType::BODY或FrameType::HIP或FrameType::WORLD

    Mat3 getJaco(LowlevelState &state, int legID); // 获取第legID条腿在state状态下的雅可比矩阵
    Vec2 getRobVelLimitX(){return _robVelLimitX;}
    Vec2 getRobVelLimitY(){return _robVelLimitY;}
    Vec2 getRobVelLimitYaw(){return _robVelLimitYaw;}
    Vec34 getFeetPosIdeal(){return _feetPosNormalStand;} // 返回各个足端中性落脚点在机身坐标系下的坐标
    double getRobMass(){return _mass;}
    Vec3 getPcb(){return _pcb;}
    Mat3 getRobInertial(){return _Ib;}

protected:
    QuadrupedLeg* _Legs[4]; // 定义四条腿的指针数组
    Vec2 _robVelLimitX; // 机器人在机身坐标系下x轴方向的速度限制
    Vec2 _robVelLimitY; // 机器人在机身坐标系下y轴方向的速度限制
    Vec2 _robVelLimitYaw; // 机器人在机身坐标系下的角速度限制
    Vec34 _feetPosNormalStand; // 代表各个足端中性落脚点在机身坐标系下的坐标
    double _mass; // 机器人简化模型的质量
    Vec3 _pcb; // 个人感觉_pcb应该是机器人质心的偏移量
    Mat3 _Ib; // 机器人简化模型在机身坐标系下的惯性矩阵
};

class A1Robot : public QuadrupedRobot{ // A1机器人类，继承自QuadrupedRobot，
public:
    A1Robot();
    ~A1Robot(){}
};

class Go1Robot : public QuadrupedRobot{
public:
    Go1Robot();
    ~Go1Robot(){};
};

#endif  // UNITREEROBOT_H