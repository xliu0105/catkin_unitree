import rospy
import torch
import rospkg
from rl_l2g.rl_gazebo_class import RL_Gazebo
import yaml
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--robot_name', type=str, default = None)  # 小写 a1 或者 go1
parser.add_argument('--REAL_ROBOT', action = 'store_true')  # 是否在真实机器人中，如果不是，则默认在gazebo仿真中
parser.add_argument('--GAMEPAD', action = 'store_true')  # 控制设备，是否是游戏手柄，如果为False，则默认为键盘控制
args_cli = parser.parse_args()



# --------------------------------- in this block, you need to implement your own get_observation function  -----------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------------------

class rsl_rl_gazebo(RL_Gazebo):  # 继承于RL_Gazebo类
  def __init__(self, **kwargs):  # 需要传入父类所需的所有参数
    super().__init__(**kwargs)  # 初始化RL_Gazebo类
  
  # NOTE: 在父类中，get_observation函数并没有被实现，因此需要在子类中实现
  def get_observation(self) -> torch.Tensor:
    
    q_obs = []
    dq_obs = []
    for i in range(12):
      q_obs.append(self._motorState[i].q)
      dq_obs.append(self._motorState[i].dq)
    
    # 所有的观测值项都需要是相同维度的，且如果是二维的，第一维度必须是1
    obs = torch.cat((torch.tensor(self.RobotCmd),  # 控制命令
                     torch.tensor(q_obs),  # 电机关节角度
                     torch.tensor(dq_obs),  # 电机关节速度
                     torch.tensor(self.imu.accelerometer) / 10.0,  # 线加速度，在训练时对这个数据进行了缩放
                     torch.tensor(self.imu.gyroscope),  # 角速度
                     torch.tensor(self.prev_action_buffer)  # 上一步的动作
                     ),dim = 0)
    return obs  # 返回观测值
    
# ---------------------------------------------------------------------------------------------------------------------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------------------

def read_yaml_with_default(filepath):  # 读取yaml文件的函数
  with open(filepath, 'r') as f:
    data = yaml.safe_load(f) or {}
    
  data.setdefault('robot_name', 'a1')
  data.setdefault('model_path', rospkg.RosPack().get_path('rl_l2gar') + '/weights/rsl_rl_model.pt')
  data.setdefault('activation', 'elu')
  data.setdefault('ctrl_mode', 10)
  data.setdefault('sim_dt', 0.002)
  data.setdefault('decimation', 4)
  data.setdefault('actionScale', 0.25)
  data.setdefault('getKeyboardCmd', True)
  data.setdefault('zAngvelLimit',0.8)
  data.setdefault('xyLinevelLimit',0.8)
  data.setdefault('device', 'cpu')
  
  return data

if __name__ == "__main__":
  
  rospy.init_node('rl_gazebo')  # 初始化节点
  
  param = read_yaml_with_default(rospkg.RosPack().get_path('rl_l2gar') + '/cfg/rsl_rl_cfg.yaml')
  if args_cli.robot_name is not None:
    param['robot_name'] = args_cli.robot_name  # 如果命令行参数中指定了robot_name，则将其传入param字典中，覆盖掉yaml文件中的robot_name
  param['REAL_ROBOT'] = args_cli.REAL_ROBOT  # 将命令行参数 REAL_ROBOT 传入param字典中
  param['GAMEPAD'] = args_cli.GAMEPAD  # 将命令行参数 GAMEPAD 传入param字典中
  print(param)
  
  s2gazebo = rsl_rl_gazebo(**param)  # 初始化rsl_rl_gazebo类
  s2gazebo.main_loop()
  rospy.loginfo("All done!")  # 输出日志信息
  
  