/************************************************************************
Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/

#ifndef _UNITREE_LEGGED_LOOP_H_
#define _UNITREE_LEGGED_LOOP_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>
#include <pthread.h>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/function.hpp>

namespace UNITREE_LEGGED_SDK 
{

constexpr int THREAD_PRIORITY    = 99;   // real-time priority

typedef boost::function<void ()> Callback;

class Loop {
public:
  // 根据GPT的回答，这里的_bindCPU应该是用来绑定这个Loop线程到某个特定的CPU核心上的，如果是-1的话表示让操作系统自由调度线程。
  // 这个_bindCPU的范围应该是0到CPU核心数-1。绑定核心可以避免线程迁移开销，同时更有利于利用核心的缓存。同时在有异构核心的系统中，不同的核心可能有不同的性能，绑定核心可以保证线程在性能较好的核心上运行。
  Loop(std::string name, float period, int bindCPU = -1):_name(name), _period(period), _bindCPU(bindCPU) {}
  ~Loop();
  void start();
  void shutdown();
  virtual void functionCB() = 0;

private:
  void entryFunc();

  std::string _name;
  float _period;
  int _bindCPU;
  bool _bind_cpu_flag = false;
  bool _isrunning = false;
  std::thread _thread;
};

class LoopFunc : public Loop {
public:
  LoopFunc(std::string name, float period, const Callback& _cb)
    : Loop(name, period), _fp(_cb){}
  LoopFunc(std::string name, float period, int bindCPU, const Callback& _cb)
    : Loop(name, period, bindCPU), _fp(_cb){}
  void functionCB() { (_fp)(); }
private:
  boost::function<void ()>  _fp;
};


}

#endif
