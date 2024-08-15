#include "FSM/State_Trotting_Custom.h"

// FIXME: 目前四足狗在静止站立时刻，机身会小幅度高频抖动，且会一直有向x轴负方向的速度，不知道是什么原因。
// FIXED: 四足的摆动腿，如果只使用PD计算力矩控制，抬腿高度会不够，因此需要再计算关节角度和角速度，这样才能保证抬腿高度足够
// FIXED: 在运动过程中还是不够稳定，要怎么去解决？尤其是在运动切换的过程，比如从前进切换到后退
// FIXED: 为了解决按空格键后所有速度控制命令都为0时可能会导致的运动速度突变的不稳定，可以在checkStepOrNot中设置，只要速度还大于某个阈值，就继续踏步。

State_Trotting_Custom::State_Trotting_Custom(CtrlComponents *ctrlComp):_dt(&(ctrlComp->dt)),FSMState(ctrlComp, FSMStateName::CUSTOM_TROTTING, "custom trotting")
{
  _robModel = ctrlComp->robotModel;
  _gait = new GaitGenerator(ctrlComp);
  Vec14 QP_weight = Vec14(50.0 , 50.0 , 1.0 , 1.0 , 1.0 , 50.0 , 20.0 , 20.0 , 1.0 , 20.0 , 20.0 , 1.0 , 0.0 , 1e-6); // 设置MPC的权重
  _mpcCtrl = new MPC_Controller(_robModel, ctrlComp, &_posFeetGlobalGoal, 0.20, QP_weight, _dt); // 初始化MPC控制器
  _est = ctrlComp->estimator;
  // _balCtrl = ctrlComp->balCtrl;
  _contact = ctrlComp->contact;
  _phase = ctrlComp->phase;
  _gaitHeight = 0.08; // 设置摆动腿的抬腿高度
#ifdef ROBOT_TYPE_Go1 // 根据机器人型号的不同，设置不同的控制PID参数
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif

#ifdef ROBOT_TYPE_A1
    _KpSwing = Vec3(400, 400, 400).asDiagonal();
    _KdSwing = Vec3(10, 10, 10).asDiagonal();
#endif
  _vxLim = _robModel->getRobVelLimitX(); // A1: 0.4
  _vyLim = _robModel->getRobVelLimitY(); // A1: 0.3
  _wyawLim = _robModel->getRobVelLimitYaw(); // A1: 0.5
}

State_Trotting_Custom::~State_Trotting_Custom()
{
  delete _gait; // _gaid是在State_Trotting_Custom类中定义的GaitGenerator类的指针，因此在析构函数中需要删除
  delete _mpcCtrl; // _mpcCtrl是在State_Trotting_Custom类中定义的MPC_Controller类的指针，因此在析构函数中需要删除
}

FSMStateName State_Trotting_Custom::checkChange()
{
  if(_lowState->userCmd == UserCommand::L2_B)
  {
    return FSMStateName::PASSIVE;
  }
  else if(_lowState->userCmd == UserCommand::L2_A)
  {
    return FSMStateName::FIXEDSTAND;
  }
  else
  {
    return FSMStateName::CUSTOM_TROTTING;
  }
}

void State_Trotting_Custom::enter()
{
  *_dt = 0.01; // 设置控制周期为0.01s，注意这里的_dt是一个指针，指向CtrlComponents类中的dt
  _mpcCtrl->reInit(11); // 每次开始时，需要调用这个函数，用于初始化一些变量
  _vCmdBody.setZero(); // 令机身在机身坐标系下的目标速度为0
  _vCmdGlobal.setZero(); // 令机身在世界坐标系下的目标速度为0
  _dYawCmd = 0; // 令目标偏航角速度为0
  _wCmdGlobal.setZero(); // 令世界坐标系下，目标角速度为0
  _yawCmd = _lowState->getYaw(); // 令目标偏航角等于当前估计的偏航角
  _Rd = rotz(_yawCmd); // 设置机身目标姿态的旋转矩阵为当前姿态的旋转矩阵
  _ctrlComp->ioInter->zeroCmdPanel(); // ioInter指针指向的命令信号清零
  _lowState->userValue.setZero(); // _lowState中的用户命令清零
  _gait->restart(); // 步态重新从起点开始
}

void State_Trotting_Custom::exit()
{
  _ctrlComp->ioInter->zeroCmdPanel(); // 将所有命令信号清零
  _lowState->userValue.setZero(); // _lowState中的用户命令清零
  _mpcCtrl->SmoothFQP_count = false;
  *_dt = _ctrlComp->dt_backup; // 恢复控制周期
  // _ctrlComp->setAllSwing(); // State_Trotting的代码中有这一句，其实没什么用
}

void State_Trotting_Custom::run()
{
  _posBody = _est->getPosition(); // 获取当前机身在世界坐标系下的位置，由估计器估计得到
  _velBody = _est->getVelocity(); // 获取当前机身在世界坐标系下的速度，由估计器估计得到
  _posFeet2BGlobal = _est->getPosFeet2BGlobal(); // 获取所有足端在世界坐标系下相对于机身中心的位置，由估计器估计得到
  _posFeetGlobal = _est->getFeetPos(); // 获取所有足端在世界坐标系下的位置，由估计器估计得到
  _velFeetGlobal = _est->getFeetVel(); // 获取所有足端在世界坐标系下的速度，由估计器估计得到
  _B2G_RotMat = _lowState->getRotMat(); // 获取当前机身坐标系到世界坐标系的旋转矩阵，直接从imu中读取并计算得到
  _G2B_RotMat = _B2G_RotMat.transpose(); // 获取当前世界坐标系到机身坐标系的旋转矩阵
  _yaw = _lowState->getYaw(); // 获取当前偏航角，直接从imu中读取并计算得到
  _dYaw = _lowState->getDYaw(); // 获取当前偏航角速度，直接从imu中读取并计算得到

  _userValue = _lowState->userValue; // 获取用户通过键盘或手柄发送的控制命令

  getUserCmd(); // 获取并处理用户的控制命令
  calcCmd();

  _velError = _velBody - _vCmdBody; // 计算速度误差

  _gait->setGait(_vCmdGlobal.segment(0,2), _wCmdGlobal(2), _gaitHeight); // _gait是一个GaitGenerator类的对象，用来计算摆线的轨迹
  _gait->run(_posFeetGlobalGoal, _velFeetGlobalGoal);

  std::cout << "_posBody: " << _posBody << std::endl;
  _forceFeetGlobalMPC = - _mpcCtrl->calF(_posBody, _yaw, _lowState->getRotMat() , _vCmdGlobal, _wCmdGlobal, _velBody, _lowState->getGyroGlobal()); 
  // 计算机器人触地腿的地反力/足端力，这里有个负号，因为MPC计算出来的是地反力，而我们需要的是足端力
  calFFeetSwing(); // 计算摆动腿的足端力
  _forceFeetBodyMPC = _G2B_RotMat*_forceFeetGlobalMPC; // 将地反力转换到机身坐标系下

  _q = vec34ToVec12(_lowState->getQ()); // 获取当前关节角度
  _tau = _robModel->getTau(_q, _forceFeetBodyMPC); // 计算关节力矩，其实就是Jocabian矩阵的转置乘以足端力

  calcQQd(); // 根据足端的目标位置和速度，计算得到该腿各个关节的角度和角速度

  if(checkStepOrNot()) // 使用checkStepOrNot()函数来判断是否要踏步，即当运动速度、位置误差、速度误差、偏航角速度等超过一定阈值时，返回true
    _ctrlComp->setStartWave(); // 调用这个函数可以切换为踏步
  else
    _ctrlComp->setAllStance(); // 调用这个函数可以切换为四足站立

  _lowCmd->setTau(_tau); // 将计算得到的关节力矩发送给低级控制器
  // _lowCmd->setQ(Eigen::Vector<double,12>::Zero()); // 如果只使用力矩控制会不稳定，即如果将关节角度设为0，只传关节力矩，会导致机器人不稳定
  // if((*_contact).sum() < 4)
  // {
  //   _lowCmd->setQ(vec34ToVec12(_qGoal)); // 将计算得到的关节角度发送给低级控制器
  //   _lowCmd->setQd(vec34ToVec12(_qdGoal)); // 将计算得到的关节角速度发送给低级控制器
  // }

  for(int i(0); i<4; ++i)
  { // 对于摆动腿来说，其目的是跟踪轨迹，因此其应接近于位置控制，对摆动腿设置setSwingGain，将刚度设为较大的数值
    if((*_contact)(i) == 0)
      _lowCmd->setSwingGain(i);
    else
      _lowCmd->setStableGain(i); // 支撑腿进行力控制，对支撑腿设置setStableGain，相对较小的刚度
  }
}

bool State_Trotting_Custom::checkStepOrNot() // 这里其实不是特别好，只根据速度来判断是否要踏步，可能速度很小，但移动的距离很大，这样就会导致机器人不踏步，但是实际上应该踏步了
{
  if( (fabs(_vCmdBody(0)) > 0.03) ||
      (fabs(_vCmdBody(1)) > 0.03) ||
      (fabs(_velBody(0)) > 0.1) || // 这里有当前速度和绕z轴角速度的判断，如果当前速度和z轴角速度超过阈值，则继续踏步
      (fabs(_velBody(1)) > 0.1) ||
      // (fabs(_posError(0)) > 0.08) || // MPC控制器中没有计算速度误差，因此这里就不用考虑位置误差了
      // (fabs(_posError(1)) > 0.08) ||
      (fabs(_velError(0)) > 0.05) ||
      (fabs(_velError(1)) > 0.05) ||
      (fabs(_dYaw) > 0.1) ||
      (fabs(_dYawCmd) > 0.20) ) // 如果机器人的速度命令、位置误差、速度误差、偏航角速度命令、当前实际速度、当前实际角速度等超过一定阈值，则返回true，机器人要切换到踏步状态
  {
    return true;
  }
  else
  {
    return false;
  }
}

void State_Trotting_Custom::getUserCmd()
{
  // Movement //
  _vCmdBody(0) = invNormalize(_userValue.ly, _vxLim(0),_vxLim(1)); // 按W和S是改变的ly，按A和D是改变的lx
  _vCmdBody(1) = -invNormalize(_userValue.lx, _vyLim(0),_vyLim(1)); // 这里的负号是因为机器人的y轴与世界坐标系的y轴方向相反
  _vCmdBody(2) = 0; // 机器人在机身坐标系下的目标速度z方向为0

  // Turning //
  _dYawCmd = -invNormalize(_userValue.rx, _wyawLim(0), _wyawLim(1)); // 按左右箭头是改变的rx
  _dYawCmd = 0.9*_dYawCmdPast + 0.1*_dYawCmd; // 角速度平滑处理
  _dYawCmdPast = _dYawCmd; // 更新上一次的目标偏航角速度
}

void State_Trotting_Custom::calcCmd()
{
  _vCmdGlobal = _B2G_RotMat * _vCmdBody; // 机器人在世界坐标系下的目标速度

  _vCmdGlobal(0) = saturation(_vCmdGlobal(0), Vec2(_velBody(0)-0.2, _velBody(0)+0.2)); // 限制_vCmdGlobal(0)的范围在_velBody(0)-0.2和_velBody(0)+0.2之间
  _vCmdGlobal(1) = saturation(_vCmdGlobal(1), Vec2(_velBody(1)-0.2, _velBody(1)+0.2)); // 限制_vCmdGlobal(1)的范围在当前速度减0.2和当前速度加0.2之间

  _vCmdGlobal(2) = 0; // z轴方向的速度始终为0

  _vCmdGlobal = 0.9*_vCmdGlobalPast + 0.1*_vCmdGlobal; // 借助历史目标速度来对速度进行平滑处理，类似滤波
  _vCmdGlobalPast = _vCmdGlobal; // 更新历史目标速度

  /* Turning */
  _yawCmd = _yawCmd + _dYawCmd * (*_dt); // 更新偏航角

  _Rd = rotz(_yawCmd); // 更新目标姿态的旋转矩阵
  _wCmdGlobal(2) = _dYawCmd; // 更新目标角速度向量，只有z轴方向有值
}

void State_Trotting_Custom::calFFeetSwing()
{
  for(int i(0);i<4;i++)
  {
    if((*_contact)(i) == 0)
    {
      _forceFeetGlobalMPC.col(i) = _KpSwing*(_posFeetGlobalGoal.col(i) - _posFeetGlobal.col(i)) + _KdSwing*(_velFeetGlobalGoal.col(i)-_velFeetGlobal.col(i));
    } // 这里并没有腿的动力学前馈，只有基于位置误差和速度误差的PD计算力矩控制，因此如果要让足抬腿高度符合要求，必须再控制关节的角度和角速度。
      // 宇树的电机允许同时发送关节力矩，角度和角速度命令，可以看宇树的教材P12页2.7.6节
  }
}

void State_Trotting_Custom::calcQQd()
{
  Vec34 _posFeet2B;
  _posFeet2B = _robModel->getFeet2BPositions(*_lowState, FrameType::BODY);

  for(int i(0);i<4;i++)
  {
    _posFeet2BGoal.col(i) = _G2B_RotMat * (_posFeetGlobalGoal.col(i) - _posBody); // 计算目标足端在机身坐标系下的位置
    _velFeet2BGoal.col(i) = _G2B_RotMat * (_velFeetGlobalGoal.col(i) - _velBody); // 计算目标足端在机身坐标系下的速度
  }

  _qGoal = vec12ToVec34(_robModel->getQ(_posFeet2BGoal, FrameType::BODY));
  _qdGoal = vec12ToVec34(_robModel->getQd(_posFeet2B, _velFeet2BGoal, FrameType::BODY));
}