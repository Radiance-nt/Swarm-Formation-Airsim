# Swarm-Formation-Airsim

The program is experimentally used, run on Ubuntu 20.04.

## 1. About

[**Swarm-Formation**](https://github.com/ZJU-FAST-Lab/Swarm-Formation) is a distributed swarm trajectory optimization
framework for formation flight in dense environments.

[**Airsim**](https://github.com/microsoft/AirSim) is a simulator for drones, cars and more, built on Unreal Engine.

## 2. Start

1. Install ROS on your machine.

2. Compile UE 4.27 (Mandatory) and Airsim Plugin on Linux, and prepare an airsim project
   following [link](https://microsoft.github.io/AirSim/unreal_custenv/).

3. Compile the project:

```
cd Swarm-Formation-Airsim
catkin_make -j1
```

4. Run Airsim simulator:
```
source devel/setup.bash
ue_config=$(rospack find airsim_package)/ue_settings/fly.json
"<path>/Binaries/Linux/<executable name>" -settings= ${ue_config}
```

5. Execute the program:

```
source devel/setup.bash
roslaunch ego_planner fly.launch
```

## 3. Tips

Strongly recommend **[rosmon](http://wiki.ros.org/rosmon)** to replace  **roslaunch**.