#include "control/MPC_Controller.h"

MPC_Controller::MPC_Controller(QuadrupedRobot *robModel, CtrlComponents *ctrlComp, Vec34 *posFeetGlobal, double Horizon, Vec14 QP_weight, double *dt)
      :_robModel(robModel),_Horizon(Horizon),_posFeetGlobal(posFeetGlobal),_ctrlComp(ctrlComp),_dt(dt)
{
  _QP_weight = QP_weight;
  _mass = robModel->getRobMass();
  _pcb = robModel->getPcb();
  _Ib = robModel->getRobInertial();
  _fricRatio = 0.4;
  _g << 0, 0, -9.81;
  F.setZero(); Fprev.setZero();
  contact = ctrlComp->contact; // contact和phase都是指向CtrlComponents类中的指针
  phase = ctrlComp->phase;
  _period = ctrlComp->waveGen->getT();
  _Tstance = ctrlComp->waveGen->getTstance();
  _Tswing = ctrlComp->waveGen->getTswing();
  _bias = ctrlComp->waveGen->getBias();

  Vec34 feetPosBody = _robModel->getFeetPosIdeal(); // 获取所有足端中性落脚点在机身坐标系下的坐标
  for(int i(0); i<4; ++i)
  {
    _feetRadius(i)    = sqrt( pow(feetPosBody(0, i), 2) + pow(feetPosBody(1, i), 2) ); // 计算机器人绕自身中心点旋转的半径，即公式中的R
    _feetInitAngle(i) = atan2(feetPosBody(1, i), feetPosBody(0, i)); // 计算大腿关节与机身中心连线到机身坐标系x轴的夹角
  }

  _fricCone_temp << 1, 0, _fricRatio, -1, 0, _fricRatio, 0, 1, _fricRatio, 0, -1, _fricRatio, 0, 0, 1; // 摩擦锥约束矩阵的临时变量，参考宇树教材P114
}

void MPC_Controller::reInit() // 每次进入MPC控制器时，需要调用这个函数，用于初始化一些变量，因为每次进入MPC控制器时，都会重新设置_dt
{
  *(_dt)=0.01; // 设置控制周期为0.01s
  _percent_dt_stance = (*_dt)/_Tstance; _percent_dt_wave = (*_dt)/_Tswing;
  _n = (int)(_Horizon/(*_dt));
  _refTraj.resize(13,_n); // 参考轨迹的大小为13*Horizon/dt的矩阵，因为在状态向量中加了z轴的重力加速度项
  _Avec.reserve(_n); _Bvec.reserve(_n); // 用于存储MPC的Horizon过程的A和B矩阵
  initMatrixA(); // 先初始化离散动力学的A矩阵不会改变的部分

  _L.resize(13*_n,13*_n); // 重新设置矩阵的大小
  _L.setZero(); // 将矩阵的值全部设置为0
  _QP_weight = Vec14(35.0 , 35.0 , 1.0 , 2.0 , 2.0 , 50.0 , 1.0 , 1.0 , 1.0 , 1.0 ,1.0 , 1.0 , 0.0 , 1e-6); // 设置MPC的权重

  for(int i(0);i<_n;i++)
  {
    _L.block(13*i,13*i,13,13) = _QP_weight.segment(0,13).asDiagonal(); // 将权重矩阵_L的对角线元素设置为QP_weight的前13个元素
  }
}

void MPC_Controller::reInit(int n) // reInit函数重载，直接指定Horizon的长度
{
  *(_dt) = 0.01;
  _percent_dt_stance = (*_dt)/_Tstance; _percent_dt_wave = (*_dt)/_Tswing;
  _n = n;
  _refTraj.resize(13,_n);
  _Avec.reserve(_n); _Bvec.reserve(_n);
  initMatrixA();

  _L.resize(13*_n,13*_n);
  _L.setZero();
  _QP_weight = Vec14(35.0 , 35.0 , 1.0 , 2.0 , 2.0 , 70.0 , 1.0 , 1.0 , 1.0 , 1.0 , 1.0 , 1.0 , 0.0 , 1e-6);
  for(int i(0);i<_n;i++)
  {
    _L.block(13*i,13*i,13,13) = _QP_weight.segment(0,13).asDiagonal();
  }
}

void MPC_Controller::calReftraj( Vec3 posBodyGlobal, double yaw, Vec3 vCmdGlobal, Vec3 wCmdGlobal)
{
  static bool standStill = false;
  if(vCmdGlobal.norm() <1e-4  && wCmdGlobal.norm() <1e-4 && standStill == false)
  {
    _pcd(0) = posBodyGlobal(0); _pcd(1) = posBodyGlobal(1);  _pcd(3) = yaw;
    _pcd(2) = (- _robModel->getFeetPosIdeal()(2,0)); // _pcd(2)前面有一个负号
    standStill = true;
  }
  else if(vCmdGlobal.norm() >1e-4  || wCmdGlobal.norm() >1e-4)
  {
    _pcd(0) = posBodyGlobal(0); _pcd(1) = posBodyGlobal(1); _pcd(2) = - _robModel->getFeetPosIdeal()(2,0);  _pcd(3) = yaw; // _pcd(2)前面有一个负号
    standStill = false;
  }
  for(int i=0; i < _n; i++) // 在Horizon的第一个时刻不是当前位置，而是当前位置的下一个时刻
  {
    _refTraj(0,i) = 0.0;
    _refTraj(1,i) = 0.0;
    _refTraj(2,i) = _pcd(3) + wCmdGlobal(2)*(i+1)*(*_dt);
    _refTraj(3,i) = _pcd(0) + vCmdGlobal(0)*(i+1)*(*_dt);
    _refTraj(4,i) = _pcd(1) + vCmdGlobal(1)*(i+1)*(*_dt);
    _refTraj(5,i) = _pcd(2);
    _refTraj(6,i) = wCmdGlobal(0);
    _refTraj(7,i) = wCmdGlobal(1);
    _refTraj(8,i) = wCmdGlobal(2);
    _refTraj(9,i) = vCmdGlobal(0);
    _refTraj(10,i) = vCmdGlobal(1);
    _refTraj(11,i) = vCmdGlobal(2);
    _refTraj(12,i) = _g(2); // z轴重力加速度项
  }
}

Vec34 MPC_Controller::calF(Vec3 posBodyGlobal, double yaw, Mat3 RotM, Vec3 vCmdGlobal, Vec3 wCmdGlobal, Vec3 velBodynow, Vec3 wBodynow)
{
  if(_ctrlComp->getWaveStatus() == WaveStatus::WAVE_ALL && _prevStatus != WaveStatus::WAVE_ALL)
  {// 在四足循环摆动的情况下，QP问题维度相对较小，因此构建QP问题和求解问题的时间相对较短，因此可以使用较长的Horizon
    _prevStatus = WaveStatus::WAVE_ALL;
    reInit();
  }
  else if(_ctrlComp->getWaveStatus() == WaveStatus::STANCE_ALL && _prevStatus != WaveStatus::STANCE_ALL)
  {// 由于在四足全都着地的情况下，由于QP问题维数较大，构建问题和求解问题的时间会长很多，因此在这种情况下，直接将Horizon的长度设置为固定小的数值；
    _prevStatus = WaveStatus::STANCE_ALL;
    reInit(11);
  }    
  auto start_time = std::chrono::high_resolution_clock::now();
  static Vec34 F;
  calReftraj(posBodyGlobal, yaw, vCmdGlobal, wCmdGlobal); // 先计算Horizon的状态变量的参考轨迹
  _phaseMPC = *phase; _contactMPC = *contact; // 用于表示MPC的Horizon过程的触地状态和步态相位，每次调用calF函数都会更新
  _footposGlobalMPC = *_posFeetGlobal; // 获取当前时刻的足端在世界坐标系下的位置坐标，主要使用当前时刻触地足的位置坐标
  calAllABmatrix(posBodyGlobal, yaw); // 计算MPC的Horizon过程中的A和B矩阵
  formQP(posBodyGlobal, yaw, RotM, vCmdGlobal, wCmdGlobal,velBodynow, wBodynow); // 构造QP问题的目标函数
  formSmoothFQP(); // 构建让地反力平滑的QP罚函数矩阵，即要让MPC的Horizon中第一个时刻的地反力和上一时刻的地反力尽量相似
  formConstraint_C(vCmdGlobal,wCmdGlobal); // 构建QP问题的约束矩阵
  formConstraint_lower_upper(); // 构建QP问题约束的上下限
  auto stop_time1 = std::chrono::high_resolution_clock::now();
  QP_solver();
  auto stop_time2 = std::chrono::high_resolution_clock::now();
  int j =0;
  for(int i(0); i<4;i++)
  {
    if((*contact)(i) == 1)
    {
      F.block(0,i,3,1) = QP_result.segment(j*3,3);
      j++;
    }
    else
    {
      F.block(0,i,3,1) = Vec3::Zero();
    }
  }
  std::cout << "F: " << F << std::endl;
  auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time1 - start_time);
  auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time2 - stop_time1);
  std::cout << "优化之前处理数据耗时: " << duration1.count() << "ms" << std::endl;
  std::cout << "QP优化求解耗时: " << duration2.count() << "ms" << std::endl;
  Fprev = vec34ToVec12(F);
  _prevContactMPC = *(contact);
  return F;
}

void MPC_Controller::calAllABmatrix(Vec3 posBodyGlobal, double yaw)
{
  // 由于机器人可能会存在四足全都在支撑和四足循环摆动的情况，因此在计算MPC中Horizon过程中的A和B矩阵需要分情况讨论，还需要做好状态切换的考虑
  _Avec.clear(); _Bvec.clear();
  if(_ctrlComp->getWaveStatus() == WaveStatus::WAVE_ALL) // 如果当前的摆动状态是：WAVE_ALL
  {
    for(int i(0); i<_n; i++) // Horizon的第一个时刻不是当前位置，而是当前位置的下一个时刻
    {
      if(i == 0)
        _Avec.push_back(resetMatrixA(yaw)); // 将MPC的Horizon过程中的A矩阵存入_Avec
      else
        _Avec.push_back(resetMatrixA(_refTraj(2,i-1))); // 将MPC的Horizon过程中的A矩阵存入_Avec
      Btemp.clear();
      for(int j(0); j<4;j++)
      {
        if(_contactMPC(j) == 1) // 如果当前这个足的触地状态是1
        {
          if(_phaseMPC(j)+_percent_dt_stance < 1) // 如果当前这个足的步态还没有达到支撑的最后时刻
          {
            _phaseMPC(j) += _percent_dt_stance; // 更新这个足的步态相位
            Btemp.push_back(calPosfeet2BGlobal(j,i,posBodyGlobal,yaw)); // 将B矩阵中关于这个足的[ri]×矩阵存入Btemp
          }
          else
          {
            Btemp.push_back(calPosfeet2BGlobal(j,i,posBodyGlobal,yaw)); // 将B矩阵中关于这个足的[ri]×矩阵存入Btemp
            _phaseMPC(j) = fmod(_phaseMPC(j)+_percent_dt_stance,1.0)*_Tstance/_Tswing; // 如果当前这个足的步态已经到了支撑的最后时刻，将步态相位对1取余数
            _contactMPC(j) = 0; // 如果当前这个足的步态已经到了支撑的最后时刻，那么就要进行腾空了
          }
        }
        else // 如果当前这个足的触地状态是0
        {
          if(_phaseMPC(j)+_percent_dt_wave <1)
          {
            _phaseMPC(j)+=_percent_dt_wave;
          }
          else
          {
            _footposGlobalMPC.col(j) = calFootposMpc(j, Vec2(_refTraj(9,i),_refTraj(10,i)), _refTraj(8,i), _phaseMPC(j), i, posBodyGlobal, yaw);
            _phaseMPC(j) = fmod(_phaseMPC(j)+_percent_dt_wave,1.0)*_Tswing/_Tstance;
            _contactMPC(j) = 1;
          }
        }
      }
      resetMatrixB(i, yaw);
      _Bvec.push_back(_B);
    }
  }
  else if(_ctrlComp->getWaveStatus() == WaveStatus::STANCE_ALL) // 如果当前的摆动状态是：STANCE_ALL
  {
    for(int i(0);i<_n;i++)
    {
      if(i == 0)
        _Avec.push_back(resetMatrixA(yaw)); // 将MPC的Horizon过程中的A矩阵存入_Avec
      else
        _Avec.push_back(resetMatrixA(_refTraj(2,i-1))); // 将MPC的Horizon过程中的A矩阵存入_Avec
      Btemp.clear();
      for(int j(0);j<4;j++)
      {
        if(_contactMPC(j) == 1) // 如果当前这个足的触地状态是1
        {
          Btemp.push_back(calPosfeet2BGlobal(j,i,posBodyGlobal,yaw)); // 将B矩阵中关于这个足的[ri]×矩阵存入Btemp
        }
        else
        {
          if(_phaseMPC(j)+_percent_dt_wave <1)
          {
            _phaseMPC(j)+=_percent_dt_wave;
          }
          else
          {
            _footposGlobalMPC.col(j) = calFootposMpc(j, Vec2(_refTraj(9,i),_refTraj(10,i)), _refTraj(8,i), _phaseMPC(j), i, posBodyGlobal, yaw);
            _phaseMPC(j) = fmod(_phaseMPC(j)+_percent_dt_wave,1.0)*_Tswing/_Tstance;
            _contactMPC(j) = 1;
          }
        }
      }
      resetMatrixB(i, yaw);
      _Bvec.push_back(_B);
    }
  }
  else if(_ctrlComp->getWaveStatus() == WaveStatus::SWING_ALL) // 如果当前的摆动状态是：SWING_ALL
  {
    // 目前MPC实现的功能还不能处理这种情况，之后需要进一步完善以应对多种步态的情况
    ROS_ERROR("当前的摆动状态是: SWING_ALL. 在MPC_Controller目前实现的功能中, 理论上不应该出现这种情况, 请检查代码逻辑");
  }
}

void MPC_Controller::formSmoothFQP() // 让当前时刻的地反力和上一时刻的地反力尽量相似，只对Horizon过程中的第一个时刻进行处理，要求其计算出来的地反力和上一时刻的地反力尽量相似
{
  if(SmoothFQP_count == false)
  {
    // _prevContactMPC == (*contact);
    _Psmooth = Eigen::MatrixXd::Zero(_P.rows(),_P.cols());
    _Qsmooth = Eigen::VectorXd::Zero(_Q.rows());
  }
  else
  {
    int _contactNum = 0;
    for(int i(0);i<4;i++) // 判断有哪些脚在当前时刻和上一时刻都是接触的
    {
      if((*contact)(i) == 1 && _prevContactMPC(i) == 1)
      {
        _contactNum++;
      }
    }
    Eigen::MatrixXd Z = 0*Eigen::MatrixXd::Identity(_contactNum*3,_contactNum*3); // FIXME: 这里暂时全都设为0了，后续需要根据实际情况进行修改
    Eigen::MatrixXd Y(_contactNum*3, _P.cols()); Y.setZero();
    Eigen::VectorXd prevF(_contactNum*3); prevF.setZero();
    int j=0,n=0;
    for(int i(0);i<4;i++)
    {
      if((*contact)(i)==1)
      {
        if(_prevContactMPC(i) == (*contact)(i))
        {
          Y.row(j*3+0) = Eigen::MatrixXd::Zero(1,_P.cols()); Y(j*3+0,n*3+0) = 1.0;
          Y.row(j*3+1) = Eigen::MatrixXd::Zero(1,_P.cols()); Y(j*3+1,n*3+1) = 1.0;
          Y.row(j*3+2) = Eigen::MatrixXd::Zero(1,_P.cols()); Y(j*3+2,n*3+2) = 1.0;
          prevF.segment(j*3,3) = Fprev.segment(i*3,3);
          j++;
        }
        n++;
      }
    }
    _Psmooth = 2*(Y.transpose())*Z*Y;
    _Qsmooth = -2*(Y.transpose())*Z*prevF; // 最前面有个负号
  }
  SmoothFQP_count = true;
}

inline Mat3 MPC_Controller::calPosfeet2BGlobal(int i, int step,  Vec3 posBodyGlobal, double yaw)
{
  if(step==0)
    return skew(_footposGlobalMPC.col(i)-posBodyGlobal-rotz(yaw)*_pcb);
  else
    return skew(_footposGlobalMPC.col(i)-_refTraj.block(3,step-1,3,1)-rotz(_refTraj(2,step-1))*_pcb);
}

Vec3 MPC_Controller::calFootposMpc(int legID, Vec2 vxyGlobal, float dYaw, float phase, int step,  Vec3 posBodyGlobal, double yaw)
{
  Vec3 _nextStep, _footPos;
  double _nextYaw;

  _nextStep(0) = vxyGlobal(0)*(1-phase)*_Tswing + vxyGlobal(0)*_Tstance/2;
  _nextStep(1) = vxyGlobal(1)*(1-phase)*_Tswing + vxyGlobal(1)*_Tstance/2;
  _nextStep(2) = 0;

  _nextYaw = dYaw*(1-phase)*_Tswing + dYaw*_Tstance/2;
  
  if(step == 0)
  {
    _nextStep(0) += _feetRadius(legID) * cos(yaw + _feetInitAngle(legID) + _nextYaw);
    _nextStep(1) += _feetRadius(legID) * sin(yaw + _feetInitAngle(legID) + _nextYaw);
    _footPos = posBodyGlobal + _nextStep;
  }
  else
  {
    _nextStep(0) += _feetRadius(legID) * cos(_refTraj(2,step-1) + _feetInitAngle(legID) + _nextYaw);
    _nextStep(1) += _feetRadius(legID) * sin(_refTraj(2,step-1) + _feetInitAngle(legID) + _nextYaw);
    _footPos = _refTraj.block(3,step-1,3,1) + _nextStep;
  }

  _footPos(2) = 0.0; 

  return _footPos;
}

void MPC_Controller::initMatrixA()
{
  _A.resize(13,13);
  _A.setZero();
  _A.block(0,0,3,3) = I3;
  _A.block(3,3,3,3) = I3;
  _A.block(6,6,3,3) = I3;
  _A.block(9,9,3,3) = I3;
  _A.block(3,9,3,3) = I3*(*_dt);
  _A(12,12) = 1;
  _A(11,12) = (*_dt);
  _A(5,12) = 0.5*(*_dt)*(*_dt);
}

inline Eigen::Matrix<double,13,13> MPC_Controller::resetMatrixA(double yaw)
{
  static Mat3 RzT = (Mat3()<<0,0,0,0,0,0,0,0,1).finished();
  RzT(0,0) = cos(yaw);
  RzT(0,1) = sin(yaw);
  RzT(1,0) = -sin(yaw);
  RzT(1,1) = cos(yaw);
  _A.block(0,6,3,3) = RzT*(*_dt);
  return _A;
}

void MPC_Controller::resetMatrixB(int step,  double yaw)
{
  #undef inverse // 为了避免和Eigen库中的inverse函数冲突，这里取消inverse的宏定义
  _B.resize(13,Btemp.size()*3); // 足式机器人的状态有13维度，最后一个维度是重力加速度_g，控制输入的维度是触地足的数量乘以3
  _B.setZero();
  for(int i(0); i< Btemp.size(); i++)
  {
    _B.block(9,i*3,3,3) = I3*(*_dt)/_mass;
    if(step ==0)
      _B.block(6,i*3,3,3) = (calIbGlobal(yaw).inverse())*Btemp[i]*(*_dt);
    else
      _B.block(6,i*3,3,3) = (calIbGlobal(_refTraj(2,step-1)).inverse())*Btemp[i]*(*_dt);
  }
  #define inverse lu_inverse // 恢复inverse的宏定义
}

inline Mat3 MPC_Controller::calIbGlobal(double yaw)
{
  static Mat3 Rz = (Mat3()<<0,0,0,0,0,0,0,0,1).finished();
  Rz(0,0) = cos(yaw);
  Rz(0,1) = -sin(yaw);
  Rz(1,0) = sin(yaw);
  Rz(1,1) = cos(yaw);
  return Rz*_Ib*(Rz.transpose());
}

void MPC_Controller::formQP( Vec3 posBodyGlobal, double yaw, Mat3 RotM,Vec3 vCmdGlobal, Vec3 wCmdGlobal, Vec3 velBodynow, Vec3 wBodynow)
{
  _Aqp.resize(_A.rows()*_n, _A.cols()); // 构建QP问题的大A矩阵
  _Aqp.setZero();
  Eigen::MatrixXd Aqp_temp = Eigen::MatrixXd::Identity(_A.rows(),_A.cols());
  for(int i(0);i<_n;i++)
  {
    Aqp_temp = _Avec[i] * Aqp_temp;
    _Aqp.block(i*_A.rows(),0,_A.rows(),_A.cols()) = Aqp_temp;
  }
  auto cal_Bveccol = [this]()->int // lambda表达式，用于计算Bvec的列数
  {
    int sum = 0;
    for(int num(0);num<_Bvec.size();num++)
    {
      sum += _Bvec[num].cols();
    }
    return sum;
  };
  _Bqp.resize(_B.rows()*_n, cal_Bveccol()); // 构建QP问题的大B矩阵
  _Bqp.setZero();
  std::vector<Eigen::MatrixXd,Eigen::aligned_allocator<Eigen::MatrixXd>> Bqp_temp(_Bvec.size()); // 用于存储Bvec的临时变量
  Bqp_temp.clear(); // 清空Bqp_temp
  int now_cols = 0; // 用于记录当前_Bqp的列数
  for(int i=0;i<_n;i++) // 这个嵌套循环用于计算_Bqp
  {
    for(int j=0;j<i+1;j++)
    {
      if(j==i)
      {
        _Bqp.block(i*_B.rows(),now_cols,_Bvec[i].rows(),_Bvec[i].cols()) = _Bvec[i]; // 注意，_A和_B的行数都是相同的
        Bqp_temp.push_back(_Bvec[i]);
        // now_cols += _Bvec[i].cols();
      }
      else
      {
        Bqp_temp[j] = _Avec[i]*Bqp_temp[j];
        _Bqp.block(i*_A.rows(),now_cols,_A.rows(),_Bvec[j].cols()) = Bqp_temp[j];
        now_cols += _Bvec[j].cols();
      }
    }
    now_cols = 0;
  }
  _K = _QP_weight[13]*Eigen::MatrixXd::Identity(_Bqp.cols(),_Bqp.cols()); // 这个MIT论文公式28的代价函数的权重矩阵，_K是控制输入的权重
  _P = 2*(_Bqp.transpose()*_L*_Bqp + _K); // QP问题的目标函数的二次项
  Eigen::Vector<double,13> _x0; // 用于存储当前时刻的状态变量_x0
  _x0.segment(0,3) = rotMatToRPY(RotM); // XXX: 这里需要格外注意，是用当前时刻的姿态角来初始化状态向量的姿态角
  _x0.segment(3,3) = posBodyGlobal; _x0.segment(6,3) = wBodynow; _x0.segment(9,3) = velBodynow; _x0(12) = _g(2);
  Eigen::VectorXd _refTrajVec(13*_n); // 将参考轨迹矩阵转换为向量
  for(int i(0);i<_n;i++)
  {
    _refTrajVec.segment(i*13,13) = _refTraj.col(i);
  }
  _Q = 2*_Bqp.transpose()*_L*(_Aqp*_x0 - _refTrajVec); // QP问题的目标函数的一次项
}

void MPC_Controller::QP_solver()
{
  _P = _P + _Psmooth; // 将平滑的QP罚函数矩阵加到QP问题的目标函数的二次项上
  _Q = _Q + _Qsmooth; // 将平滑的QP罚函数矩阵加到QP问题的目标函数的一次项上
  _Psparse = _P.sparseView(); // 将_P转换为稀疏矩阵

  // 注意，由于本问题每次求解的问题维度都是不同的，因此每次都要重新初始化求解器
  OsqpEigen::Solver solver; // 创建一个OsqpEigen的求解器
  solver.settings()->setWarmStart(true);
  solver.settings()->setVerbosity(false); // 设置求解器的输出信息为false
  solver.data()->setNumberOfVariables(_Q.rows());
  solver.data()->setNumberOfConstraints(_C.rows());
  if(!solver.data()->setGradient(_Q))
    ROS_ERROR("Failed to set gradient");
  if(!solver.data()->setLinearConstraintsMatrix(_Csparse))
    ROS_ERROR("Failed to set linear constraint matrix");
  if(!solver.data()->setHessianMatrix(_Psparse))
    ROS_ERROR("Failed to set hessian matrix");
  if(!solver.data()->setLowerBound(_l))
    ROS_ERROR("Failed to set lower bound");
  if(!solver.data()->setUpperBound(_u))
    ROS_ERROR("Failed to set upper bound");
  if(!solver.initSolver())
    ROS_ERROR("Failed to initialize solver");
  if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
    ROS_ERROR("Failed to solve the problem");
  QP_result = solver.getSolution();
}