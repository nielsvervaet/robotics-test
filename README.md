# Robotics Software Engineer Task

I was asked to design and implement a pick-and-place application using a Gazebo simulation environment. The expected result of this task is a docker image and a GitHub repo with the source code.

## Main Result

The  main result of my work on this Robotics Software Engineer task is summarized in the following animation. 

![PickAndPlace](https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExMmFmMGNjNDZmNDE1ODcyZjMwZjgwMTFhMjhlZjBkYzRkMThlMjcwNSZjdD1n/JZPQAZkFR3mmnkDC9Q/giphy.gif)

A motion planning methodology was developed by making use of the [MoveIt](https://moveit.picknik.ai/humble/index.html) robot manipulation software. The following chapter guides us to the steps taken and lessons learned that got me to this main result.

## Steps Taken

### 1) Starting situation

  The starting situation can be viewed via the following terminal commands:

  ```bash 
  $ source ${ROS1_INSTALL_PATH}/setup.bash 
  $ source catkin_ws/devel/setup.bash
  $ roslaunch simple_scene gazebo.launch
  ```
  
  In short, a Gazebo simulation of a manipulator arm is provided and the objective is to move all blocks from one table to the other. An overview can be seen in the following animation:

![Gazebo_1](https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExMjIwN2IwOTI4ZmM1ZjI3NWIxMmFmYjI4MTE2OWVkODhkZGUyNGM4MiZjdD1n/EIBFqhjWdSIOTZbTUA/giphy.gif)

### 2) Moveit2 - Ubuntu 22.04

  A literature review and my notes from the previous talks with REMY robotics pointed me at the MoveIt manipulator software that can be used for solving motion planning problems to robotic manipulators. I found a [pick and place with moveit task constructor tutorial](https://moveit.picknik.ai/humble/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html), which obviously can be a great starting point in solving this test assignment. I followed the tutorial succesfully with the following set-up:

| Ubuntu 20.04 | ROS2 Humble | Moveit2 |
|--------------|-------------|---------|


  This set-up was beneficial for me given my prior experience with CPP ROS2, allowing for fast learning due to familiar syntax. My idea is to     rewrite the pick and place tutorial for the case of our UR robot and use a ros1-bridge to communicate between the simulation and the motion     planner. I ran into the problem that [ros1_bridge on ubuntu 22.04 not straight-forward](https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html). It requires a.o. building ros and moveit from source (After a long time trying, I didn't manage to get all installations right). 
  
### 3) Moveit2 - Ubuntu 20.04

  Previously I have worked with ros1_bridges on Ubuntu 20.04 without major problems. My next step was thus to downgrade the OS while pursuing my initial strategy. The following set-up was installed:

| Ubuntu 20.04 | ROS2 Foxy | ROS1 NoEtic | Moveit2 |
|--------------|-----------|-------------|---------|
  
  With this set-up I managed to run the Gazebo Simulation in ROS1 while bridging the `\joint_states` command to a subscriber node running in ROS2 Foxy. This result is reproducable by running the following sets of terminal commands in separate terminals:

T1:

```bash 
$ source ${ROS1_INSTALL_PATH}/setup.bash
$ source catkin_ws/devel/setup.bash
$ roslaunch simple_scene gazebo.launch

```
T2: 

``` bash
$ source ${ROS1_INSTALL_PATH}/setup.bash
$ source ${ROS2_INSTALL_PATH}/setup.bash
$ export ROS_MASTER_URI=http://localhost:11311
$ ros2 run ros1_bridge dynamic_bridge
```

T3: 

``` bash
$ source ${ROS2_INSTALL_PATH}/setup.bash
$ source colcon_ws/install/setup.bash
$ ros2 launch listener_jointstates_cpp listener_jointstates.launch.py 
```
In T3, one is supposed to see the following behavior: 



The ROS2 listener node prints in the terminal the name of one of the joints in the ROS1 Gazebo simulation. Hence, it is possible to use the rosbridge as intended! Another set-up problem arose while building on this idea. The pick and place task is highly relying on the Moveit Task Constructor, which appears to be unreleased for Moveit2 of ROS2 Foxy. I have at least not been able to resolve all dependency issues that I ran into, and this is where I dropped my contributions to make this test assignment work via a ROS2 solution. 

## Goal

The scope of this assignment is limited to the robot control part of a continuous pick-and-place cycle. There are two tables; one with 5 blocks and one empty. The goal is to move all of the blocks onto the other table.

No vision/perception is needed. Poses of the blocks are accessible through the ros_service:

`/gazebo/get_model_state`

The gripper can be controlled through a topic:

`/gripper_joint_position/command`

## Build && Run

### Using docker image

```bash
$ docker pull ghcr.io/remyrobotics/robotics-test:latest
$ xhost local:root
$ docker-compose up
```

Alternatively, you can build manually with the given Dockerfile.

### Building from Source

```bash
$ ./build.sh
$ cd catkin_ws
$ catkin build
$ source devel/setup.bash
$ roslaunch simple_scene gazebo.launch
```

## Evaluation

You can use any 3rd party package. We would like to see how you design the system and the planning & control pipeline.

## Submission

A docker image and the source code are required. Please explain your design choices. If it is not possible due to circumstances, please send us source codes with clear instructions and explanations.

## Additional Questions

- How would you improve the efficiency and reliability?
- What would you do if we needed multiple robots in a system?
- How would you deploy the application to multiple physical locations? What is needed to scale it?
