/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifndef CONTROLFRAME_H
#define CONTROLFRAME_H

#include "FSM/FSM.h"
#include "control/CtrlComponents.h"

class ControlFrame{ // 控制框架类，用于控制机器人的运动，只包含一个FSM控制器和一个控制组件类，在main函数中循环调用这里的run函数，这里的run又调用FSM控制器的run函数
public:
	ControlFrame(CtrlComponents *ctrlComp);
	~ControlFrame(){
		delete _FSMController;
	}
	void run(); // 这个函数里只调用了FSM控制器的run函数
private:
	FSM* _FSMController;
	CtrlComponents *_ctrlComp;
};

#endif  //CONTROLFRAME_H