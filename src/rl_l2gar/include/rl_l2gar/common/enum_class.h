#ifndef ENUM_CLASS_H
#define ENUM_CLASS_H

// 注意这里没有用enum class，而是用enum，因此其中的各个元素能够代表整数，而不是只能代表枚举类型
enum UserCommand_rl{ // 用户命令状态，包括无效状态，开始，停止，固定站立，被动，自由站立，move_base，平衡测试，摆动测试，步态测试
  PASSIVE,  // passive保护模式
  RUNNING,
  PAUSE,
  SHUT_DOWN
};


#endif  // ENUM_CLASS_H