import rospy
import torch
import sys
import keyboard
from dataclasses import dataclass, field
import numpy as np
from unitree_legged_msgs.msg import MotorCmd, MotorState, LowCmd
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty
import threading
from pynput import keyboard
from utils.enum_utils import Control_State
import time
from rsl_rl.modules import ActorCritic, ActorCriticRecurrent, EmpiricalNormalization
from rl_l2gar.msg import LowState_rl

from utils.math_utils import clip  # custom clip function，自定义的clip函数
  
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
  def __init__(self, robot_name, model_path, net_framework, actor_hidden_dims, empirical_normalization, critic_hidden_dims, activation, REAL_ROBOT,
               GAMEPAD, num_obs, num_actions, joint_offset: list[float], ctrl_mode = 0, device = 'cpu', sim_dt = 0.005, 
               decimation = 4, use_default_offset = True, getKeyboardCmd = True, xyLinevelLimit = 0.6, zAngvelLimit = 0.8, actionScale = 0.25, 
               ctrl_kp = 25, ctrl_kd = 0.5):  # 初始化函数
    self.args = {"net_framework": net_framework, "empirical_normalization": empirical_normalization, "actor_hidden_dims": actor_hidden_dims, 
                 "critic_hidden_dims": critic_hidden_dims, "activation": activation, "ctrl_dt": sim_dt * decimation, "device": device, 
                 "joint_offset": joint_offset, "actionScale": actionScale, "xyLinevelLimit": xyLinevelLimit, "zAngvelLimit": zAngvelLimit,
                 "getKeyboardCmd": getKeyboardCmd, "ctrl_mode": ctrl_mode, "num_obs": num_obs, "num_actions": num_actions, "REAL_ROBOT": REAL_ROBOT,
                 "use_default_offset": use_default_offset, "model_path": model_path, "ctrl_kp": ctrl_kp, "ctrl_kd": ctrl_kd, "GAMEPAD": GAMEPAD}
    self._motorState = [MotorState_custom() for _ in range(12)]  # init motor state，初始化电机状态信息
    self.Cmd = [MotorCmd() for _ in range(num_actions)]  # init motor command，初始化电机命令
    self.RobotCmd = [0.0 for _ in range(3)]  # init keyboard command，初始化键盘命令，键盘命令分别为x和y轴速度以及z轴角速度
    self.Ctrl_state = Control_State.PAUSE  # control type，控制类型
    self.prev_action_buffer = [0.0 for _ in range(num_actions)]  # 当第一次运行时，上一个动作设为全0
    self.imu = IMUState()  # init imu state，初始化IMU状态信息
    self.load_model(model_path)  # load model，加载模型
    self.initSend(robot_name)  # 初始化关节控制发布器
    self.initRecv(robot_name)  # 初始化状态订阅器
    self.unpause_gazebo()  # unpause gazebo，取消暂停gazebo
    if self.args["GAMEPAD"]:
      pass
    else:
      if self.args["getKeyboardCmd"]:  # 如果getKeyboardCmd为True，就启动键盘监听器
        self.listener = keyboard.Listener(on_press=self.keyboard_on_press)
        self.listener.start()
        print("Keyboard listener started")
    self.events = threading.Event()
    self.RL_thread = threading.Thread(target=self.run_policy)  # 创建一个线程，用来运行策略
    self.Control_thread = threading.Thread(target=self.control_robot)  # 创建一个线程，用来控制机器人
    
  def keyboard_on_press(self, key):
    if hasattr(key,'char'):
      if key.char == 'q':
        self.Ctrl_state = Control_State.SHUT_DOWN
        return False
      elif key.char == 'p':
        rospy.loginfo("Pause code")
        self.Ctrl_state = Control_State.PAUSE
      elif key.char == 'f':
        rospy.loginfo("Resume code")
        self.Ctrl_state = Control_State.RUNNING
      elif key.char == 'r':
        rospy.loginfo("set Passive")
        self.Ctrl_state = Control_State.PASSIVE
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
    self.RobotCmd[0:2] = clip(self.RobotCmd[0:2], -self.args["xyLinevelLimit"], self.args["xyLinevelLimit"])
    self.RobotCmd[2] = clip(self.RobotCmd[2], -self.args["zAngvelLimit"], self.args["zAngvelLimit"])
    
  def unpause_gazebo(self):  # gazebo默认启动时是暂停的，需要取消暂停
    client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    client.wait_for_service()
    client.call()
    rospy.loginfo("Unpause gazebo successfully")  # 取消暂停gazebo
    
  def load_model(self, model_path):  # load model，加载模型
    # load actor-critic model，加载actor-critic模型
    actor_critic_class = eval(self.args["net_framework"])
    self.actor_critic : ActorCritic | ActorCriticRecurrent = actor_critic_class(
      self.args["num_obs"], self.args["num_obs"], self.args["num_actions"], self.args["actor_hidden_dims"],
      self.args["critic_hidden_dims"], self.args["activation"]
    ).to(self.args["device"])
    
    # load empirical_normalization，加载经验归一化
    self.empirical_normalization = self.args["empirical_normalization"]
    if self.empirical_normalization:
      self.obs_normalizer = EmpiricalNormalization(shape = [self.args["num_obs"]], until = 1.0e8).to(self.args["device"])
    else:
      self.obs_normalizer = torch.nn.Identity()
    
    # load model weights，加载模型权重
    loaded_dict = torch.load(model_path)
    self.actor_critic.load_state_dict(loaded_dict['model_state_dict'])
    if self.empirical_normalization:
      self.obs_normalizer.load_state_dict(loaded_dict['obs_norm_state_dict'])
      
  def get_inference_policy(self):  # get inference policy，获取推理策略
    self.eval_mode()
    self.actor_critic.to(self.args["device"])
    policy = self.actor_critic.act_inference
    if self.empirical_normalization:
      self.obs_normalizer.to(self.args["device"])
      policy = lambda obs: self.actor_critic.act_inference(self.obs_normalizer(obs))
    return policy
  
  def eval_mode(self):  # set actor and obs_normalizer to eval mode，设置actor和obs_normalizer为eval模式
    self.actor_critic.eval()
    if self.empirical_normalization:
      self.obs_normalizer.eval()
  
  def main_loop(self):  # main loop，主循环
    self.Control_thread.start()
    self.RL_thread.start()
    self.RL_thread.join()
    self.Control_thread.join()
    
  def run_policy(self):
    rate = rospy.Rate(1/self.args["ctrl_dt"])  # control rate，控制频率
    policy = self.get_inference_policy()
    while not self.Ctrl_state == Control_State.SHUT_DOWN:
      start = time.time()
      if self.Ctrl_state == Control_State.PAUSE:
        rate.sleep()
        continue
      elif self.Ctrl_state == Control_State.RUNNING:
        if self.args["net_framework"] == "ActorCritic":
          obs: torch.Tensor = self.get_observation()  # 调用get_observation函数获取观测值
          mid_time = time.time()
          if obs.dim() == 1:  # 如果obs是1D张量，就在第0维度上增加一个维度
            obs = obs.unsqueeze(0)
          if obs.dim() != 2 or obs.shape[0] != 1:  # 检查obs是否是2D张量，且第0维度是否等于1
            raise ValueError("obs must be 2D tensor and shape[0] must equal to 1")
          if obs.shape[1] != self.args["num_obs"]:  # 检查obs的第1维度是否等于num_obs
            raise ValueError("obs shape[1] must equal to num_obs")
          if self.args["device"] == 'cuda':  # 如果device是cuda，就将obs转移到cuda上
            obs = obs.to(self.args["device"])
          self.action_policy: np.ndarray = policy(obs).detach().cpu().squeeze().numpy()
          self.record_prevAction(self.action_policy)  # 记录上一个动作
        elif self.args["net_framework"] == "ActorCriticRecurrent":
          raise NotImplementedError("ActorCriticRecurrent is not supported yet")
        self.events.set()
        end = time.time()
        print("Obs time cost: ", mid_time - start)
        print("Time cost: ", end - start)
        rate.sleep()
      elif self.Ctrl_state == Control_State.PASSIVE:
        self.events.set()
        rate.sleep()
        continue
    self.events.set()  # NOTE: 这里多放一个events.set()，是为了在按下q键时，确保Control_thread也能顺利退出，否则可能会卡在control_robot函数中的events.wait()处
    rospy.loginfo("Exit run_policy thread")  # 退出run_policy，关闭节点
  
  def control_robot(self):
    while not self.Ctrl_state == Control_State.SHUT_DOWN:
      if self.Ctrl_state == Control_State.PAUSE:
        time.sleep(0.01)
        self.events.clear()
        continue
      elif self.Ctrl_state == Control_State.RUNNING or self.Ctrl_state == Control_State.PASSIVE:
        self.events.wait()
        self.events.clear()
        if self.args["use_default_offset"]:
          action = self.action_policy*self.args["actionScale"] + np.array(self.args["joint_offset"])  # 加上关节偏移
        else:
          action = self.action_policy*self.args["actionScale"]
        self.sendCommand(action)  # 发送电机命令，注意，如果在RL训练时对动作进行了缩放，这里也要进行相应的缩放
    rospy.loginfo("Exit control_robot thread")  # 退出run_policy，关闭节点
  
  def get_observation(self) -> torch.Tensor:  # NOTE: 继承时必须要重写实现这个函数，否则会报错
    raise NotImplementedError("get_observation must be implemented")
    
  def sendCommand(self, action):
    Cmd_real_robot = LowCmd()
    # IMPORTANT:Cmd_real_robot.motorCmd和self.Cmd都是列表，是可变对象，因此修改Cmd_buffer的值会同时修改self.Cmd或Cmd_real_robot.motorCmd的值
    if self.args["REAL_ROBOT"]:
      Cmd_buffer = Cmd_real_robot.motorCmd
    else:
      Cmd_buffer = self.Cmd
    if self.Ctrl_state == Control_State.PASSIVE:
      for i in range(12):
        Cmd_buffer[i].mode = 10
        Cmd_buffer[i].q = 0.0
        Cmd_buffer[i].dq = 0.0
        Cmd_buffer[i].tau = 0.0
        Cmd_buffer[i].Kp = 0.0
        Cmd_buffer[i].Kd = 8.0
    else:
      if self.args["ctrl_mode"] == 0:  # position control，位置控制
        for i in range(12):
          Cmd_buffer[i].mode = 10
          Cmd_buffer[i].q = action[i]
          Cmd_buffer[i].dq = 0.0
          Cmd_buffer[i].tau = 0.0
          Cmd_buffer[i].Kp = self.args["ctrl_kp"]  # 这里用的PD值和isaac lab中的值需要是一样的
          Cmd_buffer[i].Kd = self.args["ctrl_kd"]
      elif self.args["ctrl_mode"] == 1:  # velocity control，速度控制
        for i in range(12):
          Cmd_buffer[i].mode = 10
          Cmd_buffer[i].q = 0.0
          Cmd_buffer[i].dq = action[i]
          Cmd_buffer[i].tau = 0.0
          Cmd_buffer[i].Kp = self.args["ctrl_kp"]  # 这里用的PD值和isaac lab中的值需要是一样的
          Cmd_buffer[i].Kd = self.args["ctrl_kd"]
      elif self.args["ctrl_mode"] == 2:  # torque control，力矩控制
        for i in range(12):
          Cmd_buffer[i].mode = 10
          Cmd_buffer[i].q = 0.0
          Cmd_buffer[i].dq = 0.0
          Cmd_buffer[i].tau = action[i]
          Cmd_buffer[i].Kp = 0.0
          Cmd_buffer[i].Kd = 0.0
      elif self.args["ctrl_mode"] == 3:  # hybrid control，混合控制，暂时不支持
        raise NotImplementedError("Hybrid control is not supported yet")
    if self.args["REAL_ROBOT"]:
      self._servo_pub.publish(Cmd_real_robot)
    else:
      for i in range(12):  # 发送电机命令给机器人
        self._servo_pub[i].publish(self.Cmd[i])

  def record_prevAction(self, action: np.ndarray):  # 记录上一个动作
    self.prev_action_buffer = action.tolist()
  
  def initSend(self, robot_name):  # 初始化关节控制发布器函数
    if self.args["REAL_ROBOT"]:  # 如果是实体机器人，则话题以及订阅形式都不一样
      self._servo_pub = rospy.Publisher("/realRobot/LowCmd", LowCmd, queue_size=1)
    else:
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
    if self.args["REAL_ROBOT"]:  # 如果是实体机器人，则话题以及订阅形式都不一样
      self._servo_sub = rospy.Subscriber("/realRobot/LowState", LowState_rl, self.REAL_ROBOT_Callback, queue_size=1)
    else:
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
  
  def REAL_ROBOT_Callback(self, msg: LowState_rl):  # real robot callback，真实机器人的回调函数
    # NOTE: 实体机器人中得到的四元数也是[w, x, y, z]的形式
    self.imu.quaternion = [msg.imu.quaternion[0], msg.imu.quaternion[1], msg.imu.quaternion[2], msg.imu.quaternion[3]]
    self.imu.gyroscope = [msg.imu.gyroscope[0], msg.imu.gyroscope[1], msg.imu.gyroscope[2]]
    self.imu.accelerometer = [msg.imu.accelerometer[0], msg.imu.accelerometer[1], msg.imu.accelerometer[2]]
    for i in range(12):  # 接收到的/realRobot/LowState话题的电机状态数据，应该是已经整理好顺序的，顺序就应该是按照isaac lab的。顺序整理应该是在rl_real_bridge中完成
      self._motorState[i].q = msg.motorState[i].q
      self._motorState[i].dq = msg.motorState[i].dq
      self._motorState[i].ddq = msg.motorState[i].ddq
      self._motorState[i].tauEst = msg.motorState[i].tauEst
      self._motorState[i].mode = msg.motorState[i].mode
    if self.args["GAMEPAD"]:
      self.RobotCmd[0] = msg.userValue.lx
      self.RobotCmd[1] = msg.userValue.ly
      self.RobotCmd[2] = msg.userValue.rx
      self.RobotCmd[0:2] = clip(self.RobotCmd[0:2], -self.args["xyLinevelLimit"], self.args["xyLinevelLimit"])
      self.RobotCmd[2] = clip(self.RobotCmd[2], -self.args["zAngvelLimit"], self.args["zAngvelLimit"])
      if self.Ctrl_state != Control_State(msg.userCmd):
        self.Ctrl_state = Control_State(msg.userCmd)
        rospy.loginfo("Control state changed to: %s", self.Ctrl_state)
        
  
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
    
    