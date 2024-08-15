#include "control/MPC_Controller.h"

void MPC_Controller::formfricConstraint() // 这个按照宇树教材P114的公式构造摩擦力约束矩阵，这个约束是大于等于0
{
  auto cal_Bveccol = [this]()->int // lambda表达式，用于计算_Bvec的列数
  {
    int sum = 0;
    for(int num(0);num<_Bvec.size();num++)
    {
      sum += _Bvec[num].cols();
    }
    return sum;
  };
  int _Bveccol = cal_Bveccol();
  _fricCone.resize(5*_Bveccol/3,_Bveccol);
  _fricCone.setZero();
  for(int i(0);i<_Bveccol/3;i++)
  {
    _fricCone.block(5*i,3*i,5,3) = _fricCone_temp;
  }
}

// void MPC_Controller::balanceTorqueConstraint() // FIXME: 这个好像有逻辑错误，如果当前只有一个足落地，可能会产生一些问题。构建让地反力产生的x和y轴力矩平衡的约束矩阵
// {
//   _balanceConst.resize(2*_Bvec.size(),_P.cols()); // 约束矩阵维度初始化
//   _balanceConst.setZero(); // 约束矩阵初始化为0
//   int now_rows = 0;
//   for(int i(0);i<_Bvec.size();i++)
//   {
//     _balanceConst.block(2*i,now_rows,2,_Bvec[i].cols()) = _Bvec[i].block(6,0,2,_Bvec[i].cols()); // 将_Bqp中的z和y轴的力矩约束矩阵赋值给_balanceConst
//     now_rows += _Bvec[i].cols();
//   }
// }

void MPC_Controller::balanceTorqueConstraint() // 构建让地反力产生的x和y轴力矩平衡的约束矩阵
{
  int num = 0;
  for(int i(0);i<_Bvec.size();i++)
  {
    if(_Bvec[i].cols()>3)
    {
      num++;
    }
  }
  _balanceConst.resize(2*num,_P.cols()); // 约束矩阵维度初始化
  _balanceConst.setZero(); // 约束矩阵初始化为0
  int now_cols = 0, now_rows = 0;
  for(int i(0);i<_Bvec.size();i++)
  {
    if(_Bvec[i].cols()<=3)
    {
      now_cols += _Bvec[i].cols();
      continue;
    }
    _balanceConst.block(now_rows,now_cols,2,_Bvec[i].cols()) = _Bvec[i].block(6,0,2,_Bvec[i].cols()); // 将_Bqp中的z和y轴的力矩约束矩阵赋值给_balanceConst
    now_cols += _Bvec[i].cols();
    now_rows += 2;
  }
}

void MPC_Controller::balanceXYFConstraint(Vec3 vCmdGlobal, Vec3 wCmdGlobal) // FIXME: 构建让各个触地足端的Z轴力大致相等的约束矩阵，应该不会用，原理是错的，Z轴力不应该相等吧
{
  if(vCmdGlobal.norm() == 0 && wCmdGlobal.norm() == 0 && (*contact).sum() == 4)
  {
    _balanceXYFConst.resize(8,_P.cols());
    _balanceXYFConst.setZero();
    _balanceXYFConst(0,0) = 1.0; _balanceXYFConst(1,1) = 1.0; _balanceXYFConst(2,3) = 1.0; _balanceXYFConst(3,4) = 1.0;
    _balanceXYFConst(4,6) = 1.0; _balanceXYFConst(5,7) = 1.0; _balanceXYFConst(6,9) = 1.0; _balanceXYFConst(7,10) = 1.0;  
  }
  else
  {
    _balanceXYFConst.resize(0,_P.cols());
  }
}

void MPC_Controller::balanceZTorqueConstraint(Vec3 vCmdGlobal, Vec3 wCmdGlobal)
{
  if(vCmdGlobal.norm() == 0 && wCmdGlobal.norm() == 0 && (*contact).sum() == 4)
  {
    int num = 0;
    for(int i(0);i<_Bvec.size();i++)
    {
      if(_Bvec[i].cols()>3)
      {
        num++;
      }
    }
    _balanceZTorque.resize(2*num,_P.cols()); // 约束矩阵维度初始化
    _balanceZTorque.setZero(); // 约束矩阵初始化为0
    int now_cols = 0, now_rows = 0;
    for(int i(0);i<_Bvec.size();i++)
    {
      if(_Bvec[i].cols()<=3)
      {
        now_cols += _Bvec[i].cols();
        continue;
      }
      _balanceZTorque.block(now_rows,now_cols,2,_Bvec[i].cols()) = _Bvec[i].block(6,0,2,_Bvec[i].cols()); // 将_Bqp中的z和y轴的力矩约束矩阵赋值给_balanceConst
      now_cols += _Bvec[i].cols();
      now_rows += 2;
    }
    Eigen::MatrixXd _zeroXYFM(_P.rows(),_P.cols());
    _zeroXYFM.setZero();
    for(int i(0);i<_P.rows()/3;i++)
    {
      _zeroXYFM(3*i+2,3*i+2) = 1.0;
    }
    _balanceZTorque = _balanceZTorque*_zeroXYFM;
  }
}

void MPC_Controller::formConstraint_C(Vec3 vCmdGlobal, Vec3 wCmdGlobal)
{
  formfricConstraint(); // 构造摩擦力约束矩阵
  balanceTorqueConstraint(); // 构建让地反力产生的z和y轴力矩平衡的约束矩阵

  // ---- 在这一部分构造约束矩阵_C ----
  _C.resize(_fricCone.rows() + _balanceConst.rows()+_balanceZTorque.rows(),_P.cols()); // 约束矩阵的维度初始化
  _C.block(0,0,_fricCone.rows(),_P.cols()) = _fricCone; // 目前只有摩擦力约束矩阵
  _C.block(_fricCone.rows(),0,_balanceConst.rows(),_P.cols()) = _balanceConst; // 将让地反力产生的z和y轴力矩平衡的约束矩阵赋值给_C

  // ------------------------------
  _Csparse = _C.sparseView(); // 将_C转换为稀疏矩阵
}

void MPC_Controller::formConstraint_lower_upper()
{
  _l.resize(_C.rows()); _u.resize(_C.rows());
  _l.setZero(); _u.setZero(); // 先全都初始化为0
  _u.segment(0,_fricCone.rows()) = Eigen::VectorXd::Constant(_fricCone.rows(),1e10); // 将上限初始化为1e10，相当于无上限
  // NOTE: 下面这个约束的上下界限制x轴和y轴力矩的约束，不能限死，也不能太大，否则会导致机器人运动不稳定，±0.5是一个比较合适的数值
  if((*contact).sum() == 4)
  {
    _u.segment(_fricCone.rows(),_balanceConst.rows()) = Eigen::VectorXd::Constant(_balanceConst.rows(),5e-10);
    _l.segment(_fricCone.rows(),_balanceConst.rows()) = Eigen::VectorXd::Constant(_balanceConst.rows(),-5e-10);
  }
  else
  {
    _u.segment(_fricCone.rows(),_balanceConst.rows()) = Eigen::VectorXd::Constant(_balanceConst.rows(),7e-1);
    _l.segment(_fricCone.rows(),_balanceConst.rows()) = Eigen::VectorXd::Constant(_balanceConst.rows(),-7e-1);
  }
}