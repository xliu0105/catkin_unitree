#ifndef MPC_CONTROLLER_H
#define MPC_CONTROLLER_H

#include <chrono>
#include <ros/ros.h>
#include "CtrlComponents.h"
#include "common/unitreeRobot.h"
#include <OsqpEigen/OsqpEigen.h> // 引入OsqpEigen库
// 与Eigen相关的头文件都在common/mathTypes.h中引入了，而且最好Eigen的头文件都放在一起，否则会报错，至少我是这样的

class MPC_Controller
{
public:
  MPC_Controller(QuadrupedRobot *robModel, CtrlComponents *ctrlComp, Vec34 *posFeetGlobal, double Horizon, Vec14 QP_weight, double *dt);
  // MPC_Controller(QuadrupedRobot *robModel, Vec34 *posFeetGlobal, double Horizon, Vec2 QP_weight);
  ~MPC_Controller() = default;
  Vec34 calF(Vec3 posBodyGlobal, double yaw, Mat3 RotM, Vec3 vCmdGlobal, Vec3 wCmdGlobal, Vec3 velBodynow, Vec3 wBodynow);
  void reInit(); // 每次进入MPC控制器时，需要调用这个函数，用于初始化一些变量，因为每次进入MPC控制器时，都会重新设置_dt
  void reInit(int n); // 函数重载，用于在MPC控制器中重新设置_n
  double *_dt; // 控制周期
  bool SmoothFQP_count = false; // 用在formSmoothFQP函数中，主要用于判断这是否是第一次或重新第一次进入MPC的control
protected:
  void calReftraj(Vec3 posBodyGlobal,double yaw, Vec3 vCmdGlobal, Vec3 wCmdGlobal); // 计算状态变量的参考轨迹
  void initMatrixA(); // 初始化离散动力学的A矩阵
  void resetMatrixB(int step,  double yaw); // 由于足式机器人的步态是周期性的，触地足的数量变化所以B的维度也会发生变化，因此这个函数用于重置B矩阵的维度和值
  inline Mat3 calIbGlobal( double yaw); // 计算机器人的惯性张量在世界坐标系下的表示
  inline Eigen::Matrix<double,13,13> resetMatrixA( double yaw); // 离散动力学的A矩阵中有一个与yaw有关的部分，这个函数用于更新这部分
  Vec3 calFootposMpc(int legID, Vec2 vxyGlobal, float dYaw, float phase, int step,  Vec3 posBodyGlobal, double yaw); // 计算MPC的Horizon过程中摆动腿的落脚点
  inline Mat3 calPosfeet2BGlobal(int i, int step, Vec3 posBodyGlobal, double yaw); // 计算在世界坐标系下第i个足端相对于机身中心的位置坐标的叉乘矩阵表示，即B矩阵中的[ri]×矩阵

  void calAllABmatrix( Vec3 posBodyGlobal, double yaw); // 计算MPC的Horizon过程中的A和B矩阵
  void formQP( Vec3 posBodyGlobal, double yaw, Mat3 RotM ,Vec3 vCmdGlobal, Vec3 wCmdGlobal, Vec3 velBodynow, Vec3 wBodynow); // 构造QP问题的目标函数
  void formfricConstraint(); // 构造摩擦力约束矩阵的函数
  void formSmoothFQP(); // 构建让地反力平滑的QP矩阵
  void formConstraint_C(Vec3 vCmdGlobal, Vec3 wCmdGlobal); // 构造约束矩阵的函数
  void formConstraint_lower_upper(); // 构建QP问题的约束的上下限
  void QP_solver(); // 调用OSQP库求解QP问题
  void balanceTorqueConstraint(); // 构建让地反力产生的z和y轴力矩平衡的约束矩阵
  void balanceXYFConstraint(Vec3 vCmdGlobal, Vec3 wCmdGlobal); // 构建让各个触地足端的Z轴力大致相等的约束矩阵，应该不会用，原理是错的
  void balanceZTorqueConstraint(Vec3 vCmdGlobal, Vec3 wCmdGlobal);
  

  WaveStatus _prevStatus = WaveStatus::STANCE_ALL; // 上一时刻的步态状态
  double _Horizon; // MPC预测的时间长度
  Vec4 _pcd; // 机身在世界坐标系下的目标位置
  QuadrupedRobot *_robModel;
  CtrlComponents *_ctrlComp;
  Mat3 _Ib; // 机器人的惯性张量
  Vec4 _feetRadius, _feetInitAngle; // _feetRadius:机器人绕自身中心点旋转的半径，即公式中的R；_feetInitAngle：大腿关节与机身中心连线到机身坐标系x轴的夹角
  Vec3 _g; // 重力加速度
  Vec3 _pcb; // 个人感觉_pcb应该是机器人质心的偏移量
  double _mass, _fricRatio; // 机器人总质量，fricRatio是摩擦力系数
  Vec12 F, Fprev; // F是足端力，Fprev是上一时刻的足端力
  Eigen::MatrixXd _refTraj; // 参考轨迹
  double  _period, _Tstance, _Tswing; // period是步态周期，stancePeriod是支撑相周期, wavePeriod是摆动相周期
  double _percent_dt_stance, _percent_dt_wave; // _dt占stancePeriod的百分比，_dt占wavePeriod的百分比
  int _n; // Horizon/dt
  Vec4 _bias; // 机器人的步态偏置
  Vec4 *phase, _phaseMPC; // phase为当前机器人的步态相位，指向CtrlComponents类中的phase；_phaseMPC是MPC的Horizon过程的步态相位
  Vec34 *_posFeetGlobal;
  Vec34 _footposGlobalMPC; //储存MPC过程中，世界坐标系下立足点的位置 
  VecInt4 *contact, _contactMPC, _prevContactMPC; // contact为当前机器人的接触状态，指向CtrlComponents类中的contact；_contactMPC是MPC的Horizon过程的接触状态
  // _prevContactMPC是用来在calAllABmatrix函数中判断当前时刻当前足的接触状态是否和上一时刻一样，主要是用来判断是否需要设计让地反力平滑
  Eigen::MatrixXd _A, _B; // 离散动力学方程的A和B矩阵
  std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd>> _Avec, _Bvec; // 用于存储MPC的Horizon过程的A和B矩阵
  std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd>> Btemp; // Btemp中存储的是B矩阵中的[ri]×矩阵

  // OSQP相关内容
  // OsqpEigen::Solver solver; // 创建一个OsqpEigen的求解器
  Eigen::MatrixXd _P, _C, _R, _M; // QP问题的目标函数的二次项和一次项和约束矩阵，_R是计算_M的中间矩阵，_M = _R^T * _R，_M是让地反力平滑的代价矩阵
  Eigen::MatrixXd _Psmooth; // 用在formSmoothFQP函数中
  Eigen::VectorXd _Qsmooth; // 用在formSmoothFQP函数中
  Eigen::VectorXd _Q; // QP问题的目标函数一次项和约束矩阵
  Eigen::SparseMatrix<double> _Psparse, _Csparse; // QP问题的目标函数的二次项和约束矩阵
  Eigen::VectorXd _l, _u; // QP问题的约束的上下限
  Eigen::MatrixXd _Aqp, _Bqp; // 构建QP问题的大A和大B矩阵，需要将_A和_B经过一定的处理后才能得到
  Vec14 _QP_weight = {1.0,1.0,1.0,1.0,1.0,50.0,1.0,1.0,1.0,1.0,1.0,1.0,0.0,1e-6}; // QP问题的权重
  Eigen::MatrixXd _L, _K; // 这个MIT论文公式28的代价函数的权重矩阵，_L是跟踪误差的权重，_K是控制输入的权重
  Eigen::MatrixXd _fricCone; // 摩擦锥约束矩阵
  Eigen::MatrixXd _balanceConst; // 让地反力产生的z和y轴力矩平衡的约束矩阵
  Eigen::MatrixXd _balanceXYFConst; // 让各个触地足端的Z轴力大致相等的约束矩阵
  Eigen::MatrixXd _balanceZTorque;
  Eigen::Matrix<double,5,3> _fricCone_temp; // 摩擦锥约束矩阵的临时变量
  Eigen::VectorXd QP_result; // QP问题的求解结果储存在这个变量中
};


#endif // MPC_CONTROLLER_H