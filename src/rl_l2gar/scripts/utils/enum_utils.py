from enum import Enum, auto
  
class Control_State(Enum):  # 这个与c++中定义的enum枚举类是一样的
  PASSIVE = 0
  RUNNING = 1
  PAUSE = 2
  SHUT_DOWN = 3