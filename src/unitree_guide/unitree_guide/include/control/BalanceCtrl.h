/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef BALANCECTRL_H
#define BALANCECTRL_H

#include "common/mathTypes.h"
#include "thirdParty/quadProgpp/QuadProg++.hh"
#include "common/unitreeRobot.h"

#ifdef COMPILE_DEBUG
    #include "common/PyPlot.h"
#endif  // COMPILE_DEBUG

class BalanceCtrl{
public:
    BalanceCtrl(double mass, Mat3 Ib, Mat6 S, double alpha, double beta);
    BalanceCtrl(QuadrupedRobot *robModel);
    Vec34 calF(Vec3 ddPcd, Vec3 dWbd, RotMat rotM, Vec34 feetPos2B, VecInt4 contact); // feetPos2B是当前机身重心到各个足端的向量在世界坐标系下的表示，整合为3*4的矩阵
#ifdef COMPILE_DEBUG
    void setPyPlot(PyPlot *plot){_testPlot = plot;}
#endif  // COMPILE_DEBUG
private:
    void calMatrixA(Vec34 feetPos2B, RotMat rotM, VecInt4 contact); // feetPos2B是当前机身重心到各个足端的向量在世界坐标系下的表示，整合为3*4的矩阵
    void calVectorBd(Vec3 ddPcd, Vec3 dWbd, RotMat rotM); // ddPcd是机身的目标加速度，dWbd是机身的目标角速度，rotM是机身当前姿态的旋转矩阵
    void calConstraints(VecInt4 contact); // contact是四个足端的接触状态，1表示接触，0表示未接触
    void solveQP();

    Mat12 _G, _W, _U;
    Mat6 _S; // QP问题的权重S
    Mat3 _Ib; // 机器人的惯性张量
    Vec6 _bd; // 简化动力学方程的b_d向量
    Vec3 _g; // 重力加速度
    Vec3 _pcb; // 个人感觉_pcb应该是机器人质心的偏移量
    Vec12 _F, _Fprev, _g0T; // _F是足端力，_Fprev是上一时刻的足端力
    double _mass, _alpha, _beta, _fricRatio; // 机器人总质量，alpha和beta是QP问题的权重的权重系数，fricRatio是摩擦力系数
    Eigen::MatrixXd _CE, _CI; // QP等式约束和不等式约束矩阵
    Eigen::VectorXd _ce0, _ci0; // QP等式约束和不等式约束向量
    Eigen::Matrix<double, 6 , 12> _A; // 简化动力学方程的A矩阵
    Eigen::Matrix<double, 5 , 3 > _fricMat; // 摩擦四棱锥不等式约束矩阵

    quadprogpp::Matrix<double> G, CE, CI; // QP问题的矩阵
    quadprogpp::Vector<double> g0, ce0, ci0, x; // QP问题的向量

#ifdef COMPILE_DEBUG
    PyPlot *_testPlot;
#endif  // COMPILE_DEBUG
};

#endif  // BALANCECTRL_H