# Social Robot Navigation
This repository simulates a robot in unity simulation, which understands navigation intent from human speech and executes it.

## Contents
- Social Robot Navigation
  - [1. Introduction](#1-introduction)
  - [2. Setup](#2-setup)
  - [3. Run](#3-run)
    - [i. Setup Parameters](#i-setup-parameters)
    - [ii. Run Unity](#ii-run-unity)
    - [iii. Run Navigation](#iii-run-navigation)
    - [iv. Run Intent Recognition](#iv-run-intent-recognition)
    - [v. Run Talkback](#v-run-talkback)
  - [4. Output](#4-output)

## 1. Introduction
This repo contains the Unity simulation for hospital scenario provided by WPI HiRO Lab [Gopher-In-Unity-Simulation](https://github.com/hiro-wpi/Gopher-In-Unity-Simulation) and a freight social robot that can understand human speech and also talk back to humans. The robots main functionality is to understand human speech containing navigation commands, derive human intent from speech like go to which room, go to which person, etc and then execute the following command. More information about working and can be found [here](https://github.com/dennyboby/social_robot_navigation/tree/master/docs/document).

## 2. Setup
In order to setup the repo follow the instruction in [SETUP.md](https://github.com/dennyboby/social_robot_navigation/blob/master/SETUP.md).

## 3. Run
This section discusses the setup of parameters and running different launch files.

### i. Setup Parameters
This section discusses the basic parameters that needs to be configured and other parameters that can be configured for custom performance.
1. Freight Unity Endpoint Parameters:
  - In `catkin_ws/social_robot_navigation/ROS/src/Freight-ROS-Unity/freight_unity_endpoint/config/params.yaml` set the `ROS_IP` to the IP address of your ROS machine. Same as the one set in Unity `Robotics -> ROS Settings`.
2. Freight Intent Recognition Parameters:
  - In `catkin_ws/social_robot_navigation/ROS/src/Freight-ROS-Unity/freight_intent_recognition/config/params.yaml` set the `access_key` following the instructions in this [link](https://picovoice.ai/docs/quick-start/console-access-key/)
  - Rest all parameters need not be changed for default functionality but can be changed for custom performance.

### ii. Run Unity
This section discusses on how to run the Unity simulation.
1. If the Unity simulation is not running then run the simulation following the instructions in [setup](https://github.com/dennyboby/social_robot_navigation/blob/master/SETUP.md#3-unity-setup). Since the simulation is already setup you only follow the points 3, 6 and 7.
2. Click on the play button in the Unity simulation to start the simulation (If you click on the play button again then the simulation will stop).
3. Then you can click on `connect` button to connect to the ROS part of the simulation.

### iii. Run Navigation
This section discusses on how to run the freight navigation. In order to run navigation, open a new terminal and follow the instructions below.

    cd catkin_ws/social_robot_navigation/ROS
    source devel/setup.bash
    roslaunch freight_navigation navigation.launch

On running the launch file you will see the following output on terminal.

![](docs/img/navigation_run.png)

### iv. Run Intent Recognition
This section discusses on how to run the freight intent recognition. In order to run intent recognition, open a new terminal and follow the instructions below.

    cd catkin_ws/social_robot_navigation/ROS
    source devel/setup.bash
    roslaunch freight_intent_recognition intent.launch

### v. Run Talkback
This section discusses on how to run the freight talkback. In order to run talkback functionality, open a new terminal and follow the instructions below.

    cd catkin_ws/social_robot_navigation/ROS
    source devel/setup.bash
    roslaunch freight_talkback talkback.launch

## 4. Output
 
