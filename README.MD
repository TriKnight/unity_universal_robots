# ROS Unity and Universal Robot Digital Twin Application
## 1.Installation: 
1. Install Universal Robot ROS driver 

https://github.com/UniversalRobots/Universal_Robots_ROS_Driver

2. Following steps to setup the ROS_Unity_Integration

https://github.com/Unity-Technologies/Unity-Robotics-Hub/tree/main/tutorials/ros_unity_integration

3. Following Moveit Tutorial

https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html
https://moveit.ros.org/documentation/concepts/


## 2. Run the code in ROS
```bash
roslaunch ros_tcp_endpoint endpoint.launch
```
>Note, By default, the server_endpoint will listen on ip 0.0.0.0 (i.e. allowing all incoming addresses) and port 10000, but these settings are configurable. To override them, you can change the command to ```roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=127.0.0.1 tcp_port:=10000``` (obviously replacing 127.0.0.1 with your desired IP and 10000 with your desired port number.)

## 3.Usage with Gazebo Simulation 
There are launch files available to bringup a simulated robot - either UR5e or UR10e.  In the following the commands for the UR10e are given. For the UR5, simply replace the prefix accordingly.  

To bring up the simulated robot in Gazebo, run:

```roslaunch ur_gazebo ur10e_bringup.launch```


___MoveIt! with a simulated robot___  
Again, you can use MoveIt! to control the simulated robot.  

For setting up the MoveIt! nodes to allow motion planning run:

```roslaunch ur10e_moveit_config ur10e_moveit_planning_execution.launch sim:=true```

For starting up RViz with a configuration including the MoveIt! Motion Planning plugin run:

```roslaunch dense_grasp ur10_rviz.launch ```

___Creating the new workspace of robot___
Run the the UR10e robot with rg2 gripper workspace
```roslaunch dense_grasp ur10e_rg2_workspace.launch```

## 4. MoveGroup example 
Start RViz and MoveGroup node

Open two shells. Start RViz and wait for everything to finish loading in the first shell:
```
roslaunch ur10e_moveit_config demo.launch
```

Another shell
```
rosrun unity_universal_robots move_group_joint_state.py
```

## 5. Running Unity control UR10e Robot

___Running the ROS Driver, Robot IP:192.168.1.201___
```
roslaunch ur_robot_driver ur10e_bringup.launch robot_ip:=192.168.1.201 
```
___Running the TCP/IP server___
```
roslaunch unity_universal_robots start_tcp_ip.launch
```
___Running the Position control___
```rosrun unity_universal_robots pos_joints_control.py```