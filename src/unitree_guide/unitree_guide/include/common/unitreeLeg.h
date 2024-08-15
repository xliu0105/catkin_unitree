/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef UNITREELEG_H
#define UNITREELEG_H

#include "common/mathTypes.h"
#include "common/enumClass.h"

class QuadrupedLeg{
public:
    QuadrupedLeg(int legID, float abadLinkLength, float hipLinkLength, 
                 float kneeLinkLength, Vec3 pHip2B); // pHip2B代表从机身中心到该腿基座坐标系{0}原点的向量
    ~QuadrupedLeg(){}
    Vec3 calcPEe2H(Vec3 q); // 计算足端到腿基座坐标系{0}原点的向量坐标
    Vec3 calcPEe2B(Vec3 q); // 计算足端到机身中心的向量坐标
    Vec3 calcVEe(Vec3 q, Vec3 qd); // 计算足端速度，参考坐标系为腿基座坐标系{0}
    Vec3 calcQ(Vec3 pEe, FrameType frame); // 逆运动学，pEe为足端坐标，frame为FrameType::BODY或FrameType::HIP，返回关节角度
    Vec3 calcQd(Vec3 q, Vec3 vEe); // 逆向微分运动学，q为关节角度，vEe为足端速度，返回关节角速度
    Vec3 calcQd(Vec3 pEe, Vec3 vEe, FrameType frame); // 逆向微分运动学，pEe为足端坐标，vEe为足端速度，frame为FrameType::BODY或FrameType::HIP，返回关节角速度
    Vec3 calcTau(Vec3 q, Vec3 force); // 计算关节力矩，q为关节角度，force为足端力
    Mat3 calcJaco(Vec3 q); // 计算雅可比矩阵
    Vec3 getHip2B(){return _pHip2B;}
protected:
    float q1_ik(float py, float pz, float b2y);
    float q3_ik(float b3z, float b4z, float b);
    float q2_ik(float q1, float q3, float px, 
                float py, float pz, float b3z, float b4z);
    float _sideSign;
    const float _abadLinkLength, _hipLinkLength, _kneeLinkLength;
    const Vec3 _pHip2B;
};

class A1Leg : public QuadrupedLeg{ // A1机器人腿类，继承自QuadrupedLeg，定义其具体参数
public:
    A1Leg(const int legID, const Vec3 pHip2B):
        QuadrupedLeg(legID, 0.0838, 0.2, 0.2, pHip2B){}
    ~A1Leg(){}
};

class Go1Leg : public QuadrupedLeg{ // Go1机器人腿类，继承自QuadrupedLeg，定义其具体参数
public:
    Go1Leg(const int legID, const Vec3 pHip2B):
        QuadrupedLeg(legID, 0.08, 0.213, 0.213, pHip2B){}
    ~Go1Leg(){}
};

#endif  // UNITREELEG_H