/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef CTRLCOMPONENTS_H
#define CTRLCOMPONENTS_H

#include "message/LowlevelCmd.h"
#include "message/LowlevelState.h"
#include "interface/IOInterface.h"
#include "interface/CmdPanel.h"
#include "common/unitreeRobot.h"
#include "Gait/WaveGenerator.h"
#include "control/Estimator.h"
#include "control/BalanceCtrl.h"
#include <string>
#include <iostream>

#ifdef COMPILE_DEBUG
#include "common/PyPlot.h"
#endif  // COMPILE_DEBUG

// 控制组件类，包含大量与控制相关的类和状态变量，如控制收发接口，输入输出命令，估计器，平衡控制器，控制频率，控制器运行的平台等
struct CtrlComponents{
public:
    CtrlComponents(IOInterface *ioInter):ioInter(ioInter){
        lowCmd = new LowlevelCmd();
        lowState = new LowlevelState();
        contact = new VecInt4;
        phase = new Vec4;
        *contact = VecInt4(0, 0, 0, 0);
        *phase = Vec4(0.5, 0.5, 0.5, 0.5);
    }
    ~CtrlComponents(){
        delete lowCmd;
        delete lowState;
        delete ioInter;
        delete robotModel;
        delete waveGen;
        delete estimator;
        delete balCtrl;
#ifdef COMPILE_DEBUG
        delete plot;
#endif  // COMPILE_DEBUG
    }
    LowlevelCmd *lowCmd; // 底层控制命令
    LowlevelState *lowState; // 底层状态
    IOInterface *ioInter; // 控制接口，可选IOROS和IOSDK
    QuadrupedRobot *robotModel; // 机器人模型
    WaveGenerator *waveGen; // 足端轨迹生成器
    Estimator *estimator; // 状态估计器
    BalanceCtrl *balCtrl; // 平衡控制器

#ifdef COMPILE_DEBUG
    PyPlot *plot;
#endif  // COMPILE_DEBUG

    VecInt4 *contact; // 4维向量，代表四条腿的触地状态，0代表离地，1代表触地
    Vec4 *phase; // 4维向量，代表步态的相位，从WaveGenerator::calcWave函数结构来分析，这个phase是占触地阶段或腾空阶段的相位，不是占整个步态周期的相位

    double dt;
    double dt_backup;
    bool *running;
    CtrlPlatform ctrlPlatform;

    void sendRecv(){ // 发送接收底层命令
        ioInter->sendRecv(lowCmd, lowState);
    }

    void runWaveGen(){
        waveGen->calcContactPhase(*phase, *contact, _waveStatus);
    }

    void setAllStance(){
        _waveStatus = WaveStatus::STANCE_ALL;
    }

    void setAllSwing(){
        _waveStatus = WaveStatus::SWING_ALL;
    }

    void setStartWave(){
        _waveStatus = WaveStatus::WAVE_ALL;
    }
    
    WaveStatus getWaveStatus()
    {
        return _waveStatus;
    }

    void geneObj(){ // 创建状态估计器和平衡控制器对象
        estimator = new Estimator(robotModel, lowState, contact, phase, &dt);
        balCtrl = new BalanceCtrl(robotModel);

#ifdef COMPILE_DEBUG
        plot = new PyPlot();
        balCtrl->setPyPlot(plot);
        estimator->setPyPlot(plot);
#endif  // COMPILE_DEBUG
    }

private:
    WaveStatus _waveStatus = WaveStatus::SWING_ALL;

};

#endif  // CTRLCOMPONENTS_H