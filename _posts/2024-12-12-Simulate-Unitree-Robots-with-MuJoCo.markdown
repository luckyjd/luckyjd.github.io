---
layout: default
title:  "Simulate Unitree Robots with MuJoCo"
date:   2024-12-12
categories: Ros2
---

# Simulate Unitree Robots with MuJoCo
## _Support go2, b2, b2w, h1, go2w. ( g1 not finish)_

In This guide:
- Clone and build source code, and requirements ( sdk2_python only )
- Try to config robot model
- Run example code to control robot

## Introduction 

[![Alt text](/assets/images/unitree_mujoco_structure.png)]({{ site.baseurl }}/assets/images/unitree_mujoco_structure.png)

- Unitree mujoco simulate : for simulate 
- If using python simulate , it is required unitree_sdk2_python ( install with pip )
- If using ccp simulate , it is required unitree_sdk2 ( install with cmake )

## Clone and build 

```sh
git clone https://github.com/unitreerobotics/unitree_mujoco.git
```

### Directory Structure
    
- `simulate`: Simulator implemented based on unitree_sdk2 and mujoco (C++)
- `simulate_python`: Simulator implemented based on unitree_sdk2_python and mujoco (Python)
- `unitree_robots`: MJCF description files for robots supported by unitree_sdk2
- `terrain_tool`: Tool for generating terrain in simulation scenarios
- `example`: Example programs

### Build dependencies ( python )

#### unitree_sdk2_python 

> Note: Should install inside a venv.
> refer to : [https://github.com/unitreerobotics/unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python)

```sh
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git
cd unitree_sdk2_python
pip install -e .
```
#### mujoco-python

```sh
pip install mujoco
```

#### joystick  ( require a joystick to control)

```sh
pip install pygame
```

**Test** 

> Note: activate venv

```sh
cd simulate_python
python unitree_mujoco.py
```

You should see the mujoco simulator with Go2 model loaded.  ( Go2 by default )

```sh
cd example/python
python stand_go2.py
```
You should see Go2 stand up and then lie down.

> Note : You can control real robot with 
```sh
python3 stand_go2.py enp3s0  # where enp3s0 is name of network interface card connected to the real robot
```

### Build dependencies ( ros2 )

Should configure unitree_ros2 first ( compile with cyclone dds ): refer [install guide](https://github.com/unitreerobotics/unitree_sdk2_python)

```sh
source ~/unitree_ros2/setup.sh     # similar with source /opt/ros/rolling/setup.bash , but this is config for unitree_ros2 , not ros2
```

Now back to unitree_mujoco repo

```sh
cd example/ros2
colcon build     # run after setup env for unitree_ros2, after build successful , you will see install folder
```

Run simulation

```sh
source ~/unitree_ros2/setup_local.sh   # use local network card 
export ROS_DOMAIN_ID=1   # set domain id to match the simulation
./install/standgo2/bin/stand_go2    # run command 
```
> Note : You can control real robot with 

```sh
source ~/unitree_ros2/setup.sh   # use network interface card which connected with real robot , can edit setup.sh file content
export ROS_DOMAIN_ID=0 
./install/stand_go2/bin/stand_go2   # run command
```




