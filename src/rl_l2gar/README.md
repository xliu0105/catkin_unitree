# rl_l2gar

"l2gar" stants for "isaaclab to gazebo and real"

This package is written based on ROS1 and tested on Ubuntu20.04 | ROS Noetic. This package currently only supports Unitree Robot A1 robots and only supports controllers trained by rsl_rl.

If you want to transfer the controller of the Unitree A1 Robot trained on the isaac lab to the ROS1 Gazebo, you can do so by modifying this code.

First, you need to modify the `rsl_rl_cfg.yaml` file in the cfg folder and fill in the required content according to the parameters set during training. Then, you need to modify the `rl_gazebo.py` file in the scripts folder. Set the get_observation function of the derived class inherited from RL_Gazebo according to the configuration order of the actor's observation values ​​during training. The return value should be a torch.Tensor of [1, obs_dim] dimensions. Finally, you can compile using catkin_make and run the controller using rosrun rl_l2gar rl_gazebo.py. Don't forget to start the Gazebo launch file before this.

Once you start the `rl_gazebo.py` file, you can use keyboard to change velocity command of the robot.