import rospy
import torch
import sys
import keyboard
from dataclasses import dataclass, asdict, field
import numpy as np
from unitree_legged_msgs.msg import MotorCmd, MotorState
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty
import threading
from pynput import keyboard
from enum import Enum, auto
from rsl_rl.modules import ActorCritic, ActorCriticRecurrent, EmpiricalNormalization

# sys.path.append('src/rl_l2gar/scripts')

from utils.math_utils import clip  # custom clip function，自定义的clip函数

class state(Enum):
  Shutdown = auto()
  Running = auto()
  Pause = auto()

@dataclass
class MotorState_custom:  # Motor state dataclass，电机状态数据类
  mode: int = 10
  q: float = 0.0
  dq: float = 0.0
  ddq: float = 0.0
  tauEst: float = 0.0

@dataclass
class IMUState:  # IMU state dataclass，IMU状态数据类
  quaternion: list[float] = field(default_factory=list)
  gyroscope: list[float] = field(default_factory=list)
  accelerometer: list[float] = field(default_factory=list)

class RL_Gazebo:
  def __init__(self, robot_name, model_path, model_args_dict, num_obs, num_actions, joint_offset: list[float], ctrl_mode = 0, device = 'cpu', 
               ctrl_dt = 0.02, getKeyboardCmd = True, xyLinevelLimit = 0.6, zAngvelLimit = 0.8, actionScale = 0.25):  # 初始化函数
    self._motorState = [MotorState_custom() for _ in range(12)]  # init motor state，初始化电机状态信息
    self.code_state = state.Running  # shutdown flag，关闭标志
    self.Cmd = [MotorCmd() for _ in range(num_actions)]  # init motor command，初始化电机命令
    self.RobotCmd = [0.0 for _ in range(3)]  # init keyboard command，初始化键盘命令，键盘命令分别为x和y轴速度以及z轴角速度
    self.prev_action_buffer = [0.0 for _ in range(num_actions)]
    self.model_args_dict = model_args_dict  # model arguments dictionary，模型参数字典
    self.ctrl_dt = ctrl_dt  # control time interval，控制时间间隔
    self.device = device  # device
    self.joint_offset : list[float] = joint_offset  # joint offset，关节默认偏移
    self.actionScale = actionScale  # action scale，动作缩放
    self.xyLinevelLimit = xyLinevelLimit  # xy line velocity limit，xy线速度限制
    self.zAngvelLimit = zAngvelLimit  # z angular velocity limit，z轴角速度限制
    self.getKeyboardCmd = getKeyboardCmd  # get keyboard command，是否获取键盘命令的bool
    self.ctrl_mode = ctrl_mode  # control mode，控制模式
    self.num_obs = num_obs  # number of observations，观测值数量
    self.num_actions = num_actions  # number of actions，动作数量
    self.imu = IMUState()  # init imu state，初始化IMU状态信息
    self.load_model(model_path)  # load model，加载模型
    self.initSend(robot_name)  # 初始化关节控制发布器
    self.initRecv(robot_name)  # 初始化状态订阅器
    self.unpause_gazebo()  # unpause gazebo，取消暂停gazebo
    if self.getKeyboardCmd:  # 如果getKeyboardCmd为True，就启动键盘监听器
      self.listener = keyboard.Listener(on_press=self.keyboard_on_press)
      self.listener.start()
    self.RL_thread = threading.Thread(target=self.run_policy)  # 创建一个线程，用来运行策略
    self.Control_thread = threading.Thread(target=self.control_robot)  # 创建一个线程，用来控制机器人
    
    
  def keyboard_on_press(self, key):
    if hasattr(key,'char'):
      if key.char == 'q':
        rospy.loginfo("Exit run_policy, shutdown node")
        self.code_state = state.Shutdown
        return False
      elif key.char == 'p':
        rospy.loginfo("Pause run_policy")
        self.code_state = state.Pause
      elif key.char == 'f':
        rospy.loginfo("Resume run_policy")
        self.code_state = state.Running
      elif key.char == 'w':
        self.RobotCmd[0] += 0.02
        print("x vel: ", self.RobotCmd[0])
      elif key.char == 's':
        self.RobotCmd[0] -= 0.02
        print("x vel: ", self.RobotCmd[0])
      elif key.char == 'd':
        self.RobotCmd[1] += 0.02
        print("y vel: ", self.RobotCmd[1])
      elif key.char == 'a':
        self.RobotCmd[1] -= 0.02
        print("y vel: ", self.RobotCmd[1])
      elif key.char == 'l':
        self.RobotCmd[2] += 0.02
        print("z vel: ", self.RobotCmd[2])
      elif key.char == 'j':
        self.RobotCmd[2] -= 0.02
        print("z vel: ", self.RobotCmd[2])
    else:
      if key == keyboard.Key.space:
        self.RobotCmd = [0.0, 0.0, 0.0]
    self.RobotCmd[0:2] = clip(self.RobotCmd[0:2], -self.xyLinevelLimit, self.xyLinevelLimit)
    self.RobotCmd[2] = clip(self.RobotCmd[2], -self.zAngvelLimit, self.zAngvelLimit)
    
  
  def unpause_gazebo(self):  # gazebo默认启动时是暂停的，需要取消暂停
    client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    client.wait_for_service()
    client.call()
    rospy.loginfo("Unpause gazebo successfully")  # 取消暂停gazebo
    
  def load_model(self, model_path):  # load model，加载模型
    # load actor-critic model，加载actor-critic模型
    actor_critic_class = eval(self.model_args_dict["class_name"])
    self.actor_critic = actor_critic_class(
      self.num_obs, self.num_obs, self.num_actions, self.model_args_dict["actor_hidden_dims"],
      self.model_args_dict["critic_hidden_dims"], self.model_args_dict["activation"]
    ).to(self.device)
    
    # load empirical_normalization，加载经验归一化
    self.empirical_normalization = self.model_args_dict["empirical_normalization"]
    if self.empirical_normalization:
      self.obs_normalizer = EmpiricalNormalization(shape = [self.num_obs], until = 1.0e8).to(self.device)
    else:
      self.obs_normalizer = torch.nn.Identity()
    
    # load model weights，加载模型权重
    loaded_dict = torch.load(model_path)
    self.actor_critic.load_state_dict(loaded_dict['model_state_dict'])
    if self.empirical_normalization:
      self.obs_normalizer.load_state_dict(loaded_dict['obs_norm_state_dict'])
      
  def get_inference_policy(self):  # get inference policy，获取推理策略
    self.eval_mode()
    self.actor_critic.to(self.device)
    policy = self.actor_critic.act_inference
    if self.empirical_normalization:
      self.obs_normalizer.to(self.device)
      policy = lambda obs: self.actor_critic.act_inference(self.obs_normalizer(obs))
    return policy
  
  def eval_mode(self):  # set actor and obs_normalizer to eval mode，设置actor和obs_normalizer为eval模式
    self.actor_critic.eval()
    if self.empirical_normalization:
      self.obs_normalizer.eval()
      
  def run_policy(self):
    policy = self.get_inference_policy()
    rate = rospy.Rate(1/self.ctrl_dt)  # control rate，控制频率
    while not self.code_state == state.Shutdown:
      if self.code_state == state.Pause:
        rate.sleep()
        continue
      elif self.code_state == state.Running:
        start_time  = rospy.Time.now().to_sec()
        if self.model_args_dict["class_name"] == "ActorCritic":
          obs: torch.Tensor = self.get_observation()  # 调用get_observation函数获取观测值
          if obs.dim() == 1:  # 如果obs是1D张量，就在第0维度上增加一个维度
            obs = obs.unsqueeze(0)
          if obs.dim() != 2 or obs.shape[0] != 1:  # 检查obs是否是2D张量，且第0维度是否等于1
            raise ValueError("obs must be 2D tensor and shape[0] must equal to 1")
          if obs.shape[1] != self.num_obs:  # 检查obs的第1维度是否等于num_obs
            raise ValueError("obs shape[1] must equal to num_obs")
          # print("obs: ", obs)
          action_policy: np.ndarray = policy(obs).detach().cpu().squeeze().numpy()
          action = action_policy*self.actionScale + np.array(self.joint_offset)  # 加上关节偏移
          self.record_prevAction(action_policy)  # 记录上一个动作
          # print("action: ", action)
          self.sendCommand(action)  # 发送电机命令，注意，如果在RL训练时对动作进行了缩放，这里也要进行相应的缩放
        elif self.model_args_dict["class_name"] == "ActorCriticRecurrent":
          raise NotImplementedError("ActorCriticRecurrent is not supported yet")
        end_time = rospy.Time.now().to_sec()
        duration = end_time - start_time
        rospy.loginfo("Duration: %f", duration)
        rate.sleep()
    rospy.loginfo("Exit run_policy, shutdown node")  # 退出run_policy，关闭节点
  
  def get_observation(self) -> torch.Tensor:  # NOTE: 继承时必须要重写实现这个函数，否则会报错
    raise NotImplementedError("get_observation must be implemented")
    
  def sendCommand(self, action):
    if self.ctrl_mode == 0:  # position control，位置控制
      for i in range(12):
        self.Cmd[i].mode = 10
        self.Cmd[i].q = action[i]
        self.Cmd[i].dq = 0.0
        self.Cmd[i].tau = 0.0
        self.Cmd[i].Kp = 25.0  # 这里用的PD值和isaac lab中的值需要是一样的
        self.Cmd[i].Kd = 0.5
    elif self.ctrl_mode == 1:  # velocity control，速度控制
      for i in range(12):
        self.Cmd[i].mode = 10
        self.Cmd[i].dq = action[i]
        self.Cmd[i].q = 0.0
        self.Cmd[i].tau = 0.0
        self.Cmd[i].Kp = 0.0
        self.Cmd[i].Kd = 1.0
    elif self.ctrl_mode == 2:  # torque control，力矩控制
      for i in range(12):
        self.Cmd[i].mode = 10
        self.Cmd[i].tau = action[i]
        self.Cmd[i].q = 0.0
        self.Cmd[i].dq = 0.0
        self.Cmd[i].Kp = 0.0
        self.Cmd[i].Kd = 0.0
    elif self.ctrl_mode == 3:  # hybrid control，混合控制，暂时不支持
      raise NotImplementedError("Hybrid control is not supported yet")
    for i in range(12):  # 发送电机命令给机器人
      self._servo_pub[i].publish(self.Cmd[i])

  def record_prevAction(self, action: np.ndarray):  # 记录上一个动作
    self.prev_action_buffer = action.tolist()
  
  def initSend(self, robot_name):  # 初始化关节控制发布器函数
    self._servo_pub = []
    self._servo_pub.append(rospy.Publisher("/" + robot_name + "_gazebo/FL_hip_controller/command", MotorCmd, queue_size=1))
    self._servo_pub.append(rospy.Publisher("/" + robot_name + "_gazebo/FR_hip_controller/command", MotorCmd, queue_size=1))
    self._servo_pub.append(rospy.Publisher("/" + robot_name + "_gazebo/RL_hip_controller/command", MotorCmd, queue_size=1))
    self._servo_pub.append(rospy.Publisher("/" + robot_name + "_gazebo/RR_hip_controller/command", MotorCmd, queue_size=1))
    self._servo_pub.append(rospy.Publisher("/" + robot_name + "_gazebo/FL_thigh_controller/command", MotorCmd, queue_size=1))
    self._servo_pub.append(rospy.Publisher("/" + robot_name + "_gazebo/FR_thigh_controller/command", MotorCmd, queue_size=1))
    self._servo_pub.append(rospy.Publisher("/" + robot_name + "_gazebo/RL_thigh_controller/command", MotorCmd, queue_size=1))
    self._servo_pub.append(rospy.Publisher("/" + robot_name + "_gazebo/RR_thigh_controller/command", MotorCmd, queue_size=1))
    self._servo_pub.append(rospy.Publisher("/" + robot_name + "_gazebo/FL_calf_controller/command", MotorCmd, queue_size=1))
    self._servo_pub.append(rospy.Publisher("/" + robot_name + "_gazebo/FR_calf_controller/command", MotorCmd, queue_size=1))
    self._servo_pub.append(rospy.Publisher("/" + robot_name + "_gazebo/RL_calf_controller/command", MotorCmd, queue_size=1))
    self._servo_pub.append(rospy.Publisher("/" + robot_name + "_gazebo/RR_calf_controller/command", MotorCmd, queue_size=1))
  
  def initRecv(self, robot_name):  # 初始化状态订阅器函数
    self._servo_sub = []
    self._imu_sub = rospy.Subscriber("/trunk_imu", Imu, self._imuCallback, queue_size=1)
    self._servo_sub.append(rospy.Subscriber("/" + robot_name + "_gazebo/FL_hip_controller/state", MotorState, self._FRhipCallback, queue_size=1))
    self._servo_sub.append(rospy.Subscriber("/" + robot_name + "_gazebo/FR_hip_controller/state", MotorState, self._FRthighCallback, queue_size=1))
    self._servo_sub.append(rospy.Subscriber("/" + robot_name + "_gazebo/RL_hip_controller/state", MotorState, self._FRcalfCallback, queue_size=1))
    self._servo_sub.append(rospy.Subscriber("/" + robot_name + "_gazebo/RR_hip_controller/state", MotorState, self._FLhipCallback, queue_size=1))
    self._servo_sub.append(rospy.Subscriber("/" + robot_name + "_gazebo/FL_thigh_controller/state", MotorState, self._FLthighCallback, queue_size=1))
    self._servo_sub.append(rospy.Subscriber("/" + robot_name + "_gazebo/FR_thigh_controller/state", MotorState, self._FLcalfCallback, queue_size=1))
    self._servo_sub.append(rospy.Subscriber("/" + robot_name + "_gazebo/RL_thigh_controller/state", MotorState, self._RRhipCallback, queue_size=1))
    self._servo_sub.append(rospy.Subscriber("/" + robot_name + "_gazebo/RR_thigh_controller/state", MotorState, self._RRthighCallback, queue_size=1))
    self._servo_sub.append(rospy.Subscriber("/" + robot_name + "_gazebo/FL_calf_controller/state", MotorState, self._RRcalfCallback, queue_size=1))
    self._servo_sub.append(rospy.Subscriber("/" + robot_name + "_gazebo/FR_calf_controller/state", MotorState, self._RLhipCallback, queue_size=1))
    self._servo_sub.append(rospy.Subscriber("/" + robot_name + "_gazebo/RL_calf_controller/state", MotorState, self._RLthighCallback, queue_size=1))
    self._servo_sub.append(rospy.Subscriber("/" + robot_name + "_gazebo/RR_calf_controller/state", MotorState, self._RLcalfCallback, queue_size=1))
  
  def _imuCallback(self, msg: Imu):  # imu的回调函数
    self.imu.quaternion = [msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z]
    self.imu.gyroscope = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
    self.imu.accelerometer = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]
  
  def _FRhipCallback(self, msg: MotorState):  # 之后的12个函数都是关节状态的回调函数
    self._motorState[0].mode = msg.mode
    self._motorState[0].q = msg.q
    self._motorState[0].dq = msg.dq
    self._motorState[0].tauEst = msg.tauEst
  
  def _FRthighCallback(self, msg):
    self._motorState[1].mode = msg.mode
    self._motorState[1].q = msg.q
    self._motorState[1].dq = msg.dq
    self._motorState[1].tauEst = msg.tauEst
  
  def _FRcalfCallback(self, msg):
    self._motorState[2].mode = msg.mode
    self._motorState[2].q = msg.q
    self._motorState[2].dq = msg.dq
    self._motorState[2].tauEst = msg.tauEst
    
  def _FLhipCallback(self, msg):
    self._motorState[3].mode = msg.mode
    self._motorState[3].q = msg.q
    self._motorState[3].dq = msg.dq
    self._motorState[3].tauEst = msg.tauEst
    
  def _FLthighCallback(self, msg):
    self._motorState[4].mode = msg.mode
    self._motorState[4].q = msg.q
    self._motorState[4].dq = msg.dq
    self._motorState[4].tauEst = msg.tauEst
    
  def _FLcalfCallback(self, msg):
    self._motorState[5].mode = msg.mode
    self._motorState[5].q = msg.q
    self._motorState[5].dq = msg.dq
    self._motorState[5].tauEst = msg.tauEst
    
  def _RRhipCallback(self, msg):
    self._motorState[6].mode = msg.mode
    self._motorState[6].q = msg.q
    self._motorState[6].dq = msg.dq
    self._motorState[6].tauEst = msg.tauEst
    
  def _RRthighCallback(self, msg):
    self._motorState[7].mode = msg.mode
    self._motorState[7].q = msg.q
    self._motorState[7].dq = msg.dq
    self._motorState[7].tauEst = msg.tauEst
    
  def _RRcalfCallback(self, msg):
    self._motorState[8].mode = msg.mode
    self._motorState[8].q = msg.q
    self._motorState[8].dq = msg.dq
    self._motorState[8].tauEst = msg.tauEst
    
  def _RLhipCallback(self, msg):
    self._motorState[9].mode = msg.mode
    self._motorState[9].q = msg.q
    self._motorState[9].dq = msg.dq
    self._motorState[9].tauEst = msg.tauEst
    
  def _RLthighCallback(self, msg):
    self._motorState[10].mode = msg.mode
    self._motorState[10].q = msg.q
    self._motorState[10].dq = msg.dq
    self._motorState[10].tauEst = msg.tauEst
    
  def _RLcalfCallback(self, msg):
    self._motorState[11].mode = msg.mode
    self._motorState[11].q = msg.q
    self._motorState[11].dq = msg.dq
    self._motorState[11].tauEst = msg.tauEst
    
    