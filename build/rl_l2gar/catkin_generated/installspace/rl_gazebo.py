import rospy
import torch
import rospkg
from rl_gazebo_class import RL_Gazebo
from dataclasses import asdict, dataclass
import yaml

# --------------------------------- in this block, you need to implement your own get_observation function  -----------------------------------
# ---------------------------------------------------------------------------------------------------------------------------------------------

class rsl_rl_gazebo(RL_Gazebo):  # 继承于RL_Gazebo类
  def __init__(self, **kwargs):  # 需要传入父类所需的所有参数
    super().__init__(**kwargs)  # 初始化RL_Gazebo类
  
  # NOTE: 在父类中，get_observation函数并没有被实现，因此需要在子类中实现
  def get_observation(self) -> torch.Tensor:
    obs = torch.empty(1, self.args["num_obs"])  # 创建一个空的张量
    obs[0,0:3] = torch.tensor(self.RobotCmd)  # 控制命令
    obs[0,3:15] = torch.tensor([self._motorState[i].q for i in range(12)])  # 电机关节角度
    obs[0,15:27] = torch.tensor([self._motorState[i].dq for i in range(12)])  # 电机关节速度
    obs[0,27:30] = torch.tensor(self.imu.accelerometer) / 10.0  # 线加速度，在训练时对这个数据进行了缩放
    obs[0,30:33] = torch.tensor(self.imu.gyroscope)  # 角速度
    obs[0,33:45] = torch.tensor(self.prev_action_buffer)  # 上一步的动作
    
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
  print(param)
  
  s2gazebo = rsl_rl_gazebo(**param)  # 初始化rsl_rl_gazebo类
  s2gazebo.main_loop()
  rospy.loginfo("All done!")  # 输出日志信息
  
  