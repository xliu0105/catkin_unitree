This workspace was created by Xu Liu.

If you want to use this ws, you need to check the file `unitree_ros/unitree_gazebo/worlds/stairs.world`, at the end of the file:

```
<include>
    <uri>model://path_to_catkin_ws/src/unitree_ros/unitree_gazebo/worlds/building_editor_models/stairs</uri>
</include>
```

Please change the path of `building_editor_models/stairs` to the real path on your PC.

And you alse need to check the path `UNITREE_LEGGED_SDK_PATH` in file `unitree_ros/unitree_ros_to_real/unitree_legged_real/CMakeLists.txt`.

Added a Custom MPC robot controller. The code is a reproduction of paper: "Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control"

Added rl_l2gar package, transferring the Unitree robot controller trained in the isaac lab to Gazebo. Currently only Unitree A1 robots are supported. If you need more detail of this package, check the README file and the code of rl_l2gar package. 