# README.md

## Starting command

This project is for KAIST ME400 Capstone Design course. We designed an Unmanned Ball Pick-And-Delivery robot for a specific map on virtual environment.
The simulator is CoppeliaSim(https://www.coppeliarobotics.com/), and ROS Noetic is required to simulate.
To simulate, turn on CoppeliaSim and adopt the scene **map_ver5_final.ttt**. If there isn't any robot in the scene, adopt the model **coppeliasim_models/teamE_robot_ver6_final.ttm**. Make sure that the robot model is associated with the script **Lua_script_ball.txt**. Use **Vortex, very accurate, dt = 50ms** as the simulator environment setting. Finally, type a command below on the terminal to activate the robot.

```bash
roslaunch data_integrate capstone_launch.launch
```


## Required library list

This package uses basic libraries such as std::msgs, opencv, etc.

Perhaps all the packages are included in the ros full package, so if the simulation environment has a full ROS noetic package, no additional dependency for ROS is required.

The simulator used in this project is CoppeliaSim(https://www.coppeliarobotics.com/). Vortex version is needed.
