
# denso_cobotta_ros2
This repository demonstrates cobotta Moveit2 in ROS2 and Rviz2.

<br>

## Requirements & Environment
* cobotta
    * OS: **Standard OS**, not COBOTTA driver for Linux
    * Version: 2.7.x ~
    * IP: 192.168.0.2

* Laptop Computer
    * OS: Ubuntu 20.04 / 22.04
    * ROS2 distributions: Foxy / Humble
    * IP: 192.168.0.3


If you want to run it on ROS2 **Humble**, please change the launch file of the moveit package.
```
Foxy: cmd=["ros2 run controller_manager spawner.py {}".format(controller)],
Humble: cmd=["ros2 run controller_manager spawner {}".format(controller)],
```

<br>

## Installation
Build from source and install ROS environments. Please see ROS Wiki.


Install ros-packages
```
sudo apt-get install ros-$ROS_DISTRO-joint-state-publisher*
sudo apt install ros-$ROS_DISTRO-robot-state-publisher
sudo apt-get install ros-$ROS_DISTRO-ros2-control*
sudo apt-get install ros-$ROS_DISTRO-moveit*
```


Create ros2 workspace
```
mkdir -p ~/cobotta_ws/src
```


Download the packages for denso-cobotta using git.
```
cd ~/cobotta_ws/src
git clone https://github.com/tasada038/denso_cobotta_ros2.git
cd..
colcon build
```

<br>

Also, **download the stl file** used in the cobotta_description package from "DENSO ROBOT MEMBER" and rename it as follows.

stl file path: **denso_cobotta_ros2/cobotta_description/meshes/**

[CAD model download URL](https://www.denso-wave.com/ja/robot/download/cad/)

<br>

`This package does not include stl files, so you will not be able to run the demonstrations unless you have downloaded it.`

<br>

- base_link.stl
- link_J1_1.stl
- link_J2_1.stl
- link_J3_1.stl
- link_J4_1.stl
- link_J5_1.stl
- link_J6_1.stl
- left_gripper_1.stl
- right_gripper_1.stl

<br>

## Setup COBOTTA hardware
Before using this package,it is necessary to change the setting of **Executable Token to Ethernet** and **CALSET startup parameters** on the Virtual TP side.

<br>

To set Executable Token to Ethernet, press 

[F6 Setting] - [F5 Communication and Token] - [F1 Executable Token]

from the top menu of the teach pendant to display the parameter list.

<br>

To set the CALSET startup parameters, press 

[F2 Arm] - [F6 Aux] - [F1 Config]

from the top menu of the teach pendant to display the parameter list.

<br>

- Executable Token : Ethernet
- CALSET on start-up : 1 (DoNot)

<br>

## Usage

### auto calset
```
cd ./cobotta_ws
python3 src/denso_cobotta_ros2/cobotta_control/cobotta_control/auto_calset.py
```


Uncomment the following if you use version 2.8 or higher

auto_calset.py line 71
```
self.m_bcapclient.robot_execute(hRobot, "ManualResetPreparation", "")
```

<br>

### Read joint param & view rviz2
ros2 node for reading robot joint angles in real time

```
cd ~/cobotta_ws
. install/setup.bash
ros2 run cobotta_control to_rviz_node
```

<br>

### Controlling cobotta with robot state demo
Display a cobotta robot model on RViz2 and controlling cobotta hardware.

Open two shells. In the first shell,convert rviz2 parameters to cobotta joint parameters with "to_cobotta_node"
```
ros2 run cobotta_control to_cobotta_node
```


In the second shell, run the launch file:
```
ros2 launch cobotta_bringup denso_cobotta_bringup.launch.py
```

<br>

### moveit demo
MoveIt2 planning demo wiht RViz2 sim and cobotta hardware.


Open two shells. In the first shell,convert rviz2 parameters to cobotta joint parameters with "to_cobotta_node"
```
ros2 run cobotta_control to_cobotta_node
```


In the second shell, run the moveit demo launch file:
```
ros2 launch cobotta_moveit_config moveit_demo.launch.py
```


In the **Planning Request**, you can change the **Planning Group** to two types:
- arm_controller

- gripper_controller

<br>

### move group demo


Open three shells. In the first shell,convert rviz2 parameters to cobotta joint parameters with "to_cobotta_node"

```
ros2 run cobotta_control to_cobotta_node
```

In the second shell, run the move group launch file:
```
ros2 launch cobotta_run_move_group move_group.launch.py
```


In the third shell, run the move group interface launch file:
```
ros2 launch cobotta_run_move_group move_group_interface.launch.py
```

### moveit cpp demo


Open two shells. In the first shell,convert rviz2 parameters to cobotta joint parameters with "to_cobotta_node"

```
ros2 run cobotta_control to_cobotta_node
```

In the second shell, run the moveit cpp launch file:

```
ros2 launch cobotta_run_moveit_cpp moveit_cpp.launch.py
```

<br>

## About denso cobotta ros2 packages

### cobotta_bringup

This package includes launch files for startup of cobotta.


### cobotta_description

This package includes configuration files for Rviz2  


### cobotta_control
This package controls the cobotta of real machine and the simulator in cooperation.

Also, orin_bcap_python is used to control cobotta.  

[bcap python package](https://github.com/DENSORobot/orin_bcap/tree/master/Python/bCAPClient)


### cobotta_moveit_config
This package includes configuration files for MoveIt2.  


### cobotta_run_move_group
This package for moving cobotta using Move Group C++ Interface.


### cobotta_run_moveit_cpp
This package for moving cobotta using MoveItCpp.

<br>

## License
This repository is licensed under the MIT license, see LICENSE.
