#ifndef TROTTING_CUSTOM_H
#define TROTTING_CUSTOM_H

#include "FSMState.h"
#include "Gait/GaitGenerator.h"
#include "control/MPC_Controller.h"

class State_Trotting_Custom : public FSMState
{
public:
  State_Trotting_Custom(CtrlComponents *ctrlComp);
  ~State_Trotting_Custom();
  void enter();
  void run();
  void exit();
  FSMStateName checkChange();

private:
  void getUserCmd();
  void calcCmd();
  bool checkStepOrNot();
  void calFFeetSwing(); // 用来计算摆动腿的关节力矩
  void calcQQd();

  QuadrupedRobot *_robModel;
  GaitGenerator *_gait;
  Estimator *_est;
  // BalanceCtrl *_balCtrl;
  // MPC_Controller *_mpcCtrl;

  // Rob State
  Vec3 _posBody, _velBody; // 当前机身在世界坐标系下的位置和速度
  double _yaw, _dYaw; // 当前机身在世界坐标系下的偏航角和偏航角速度
  Vec34 _posFeetGlobal, _velFeetGlobal; // 当前足端在世界坐标系下的位置坐标和速度
  Vec34 _posFeet2BGlobal; // 当前足端在世界坐标系下相对于机身中心的位置坐标
  RotMat _B2G_RotMat, _G2B_RotMat; // 当前机身坐标系到世界坐标系的旋转矩阵，当前世界坐标系到机身坐标系的旋转矩阵
  Vec12 _q; // 各个关节的角度

  // Command
  // Vec3 _pcd; // 机身在世界坐标系下的目标位置，这个其实在MPC中用不到吧
  Vec3 __ddPcd, _dWbd; // 机身在世界坐标系下的目标线加速度和角加速度
  Vec3 _vCmdGlobal, _vCmdBody; // 机身在世界坐标系下的目标速度，机身在机身坐标系下的目标速度
  double _yawCmd, _dYawCmd; // _yawCmd是在世界坐标系下的目标偏航角，_dYawCmd是在世界坐标系下的目标偏航角速度
  double _dYawCmdPast = 0; // 上一次的目标偏航角速度(z轴角速度)
  Vec3 _vCmdGlobalPast = Vec3::Zero(); // 上一次的目标速度
  RotMat _Rd; // 机身目标姿态的旋转矩阵
  Vec3 _wCmdGlobal; // 机器人在世界坐标系下的目标角速度向量
  Vec34 _posFeetGlobalGoal, _velFeetGlobalGoal; // 足端在世界坐标系下的目标位置坐标和目标速度
  Vec34 _posFeet2BGoal, _velFeet2BGoal; // 足端在机身坐标系下相对于机身中心的目标位置坐标和目标速度
  Vec12 _tau; // 各个关节的前馈力矩
  Vec34 _qGoal, _qdGoal; // 各个关节的目标角度和目标角速度
  double *_dt; // 控制周期


  // Control Parameters
  VecInt4 *_contact; // 4维向量，代表四条腿的触地状态，0代表离地，1代表触地
  Vec3 _posError, _velError; // 当前时刻机身在世界坐标系下的位置和速度误差
  Vec4 *_phase; // 机器人的步态相位，指向CtrlComponents类中的phase
  Mat3 _KpSwing, _KdSwing; // 摆动足端修正力的稀疏
  double _gaitHeight; // 摆动腿轨迹的抬腿高度
  Vec2 _vxLim, _vyLim, _wyawLim; // 机身坐标系下x,y方向速度和z轴角速度的限制
  Vec34 _forceFeetGlobalMPC, _forceFeetBodyMPC; // MPC计算出来的机器人的世界坐标系下的地反力变量和body坐标系下的地反力变量

};


#endif // TROTTING_CUSTOM_H