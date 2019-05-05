This is Zhaoyuan's tutorial of how to use UR arm. 

Updated on 5/4/19.

Tested on UR5e in Biorobotics Lab.

1. clone https://github.com/guzhaoyuan/universal_robot/tree/gzy, into ros workspace. use branch gzy

2. clone https://github.com/guzhaoyuan/ur_modern_driver/tree/gzy, into ros workspace. use branch gzy

3. install ros-kinetic related dependancies if complie fails. As far as I know, at least these package should be installed

```
sudo apt install ros-kinetic-moveit-core industrial_msgs moveit_kinematics moveit_visual_tools moveit_ros_planning_interface ros-kinetic-controller-manager ros-kinetic-moveit-ros-control-interface sudo apt install ros-kinetic-moveit-planners-ompl ros-kinetic-moveit-ros-visualization ros-kinetic-moveit-commander
```

4. check the ip of the robot in the teach pendant, in my example the ip is 10.10.10.61, ping the robot to make sure you could connect to the robot

```
ping 10.10.10.61
```

5. open the ur_modern_driver.

```
roslaunch ur_modern_driver ur5e_bringup.launch robot_ip:=10.10.10.61
```

6. in another terminal, 

```
rostopic echo /joint_states to see joint angles
```

if the joint numbers are keep updating, then the driver part is done. Next you have several options.

7. First try control the robot using URscript from command line, it is quite handy

```
rostopic pub /ur_driver/URScript std_msgs/String "movej([0,-1.57,0,-1.57,0,0],a=30, v=20, t=0, r=0)"
```

8. read script manual to understand how to set up the system and send approporate control. Basically, all commands in Chapter 2 of the script manual can be sent to the robot using this cmd method

9. try "test_move.py". It use /follow_joint_trajectory interface to send a joint trajectory and then follows it, in another terminal:

```
cd ur_modern_driver
python test_move.py
```

10. Use moveit to plan and control, in another terminal:

```
roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch
```

try play using moveit

11. Use a python script to talk to moveit, plan and control using python. So with the moveit window in step 10 open, in another terminal:

```
cd universal_robot/ur5_e_moveit_config/scripts
python planning_scene.py
```

play follow the instruction in the terminal.

12. So a complete workflow to this point is

	A. power on the robot, make sure its network address is 10.10.10.61, and you could ping the robot

	B. roslaunch ur_modern_driver ur5e_bringup.launch robot_ip:=10.10.10.61
	
	C. roslaunch ur5_e_moveit_config ur5_e_moveit_planning_execution.launch
  
	D. python planning_scene.py
