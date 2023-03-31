# Robotics Software Engineer Task

I was asked to design and implement a pick-and-place application using a Gazebo simulation environment. The expected result of this task is a docker image and a GitHub repo with the source code.

## Main Result

The  main result of my work on this Robotics Software Engineer task is summarized in the following animation. 

![PickAndPlace](https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExMmFmMGNjNDZmNDE1ODcyZjMwZjgwMTFhMjhlZjBkYzRkMThlMjcwNSZjdD1n/JZPQAZkFR3mmnkDC9Q/giphy.gif)

A motion planning methodology was developed by making use of the [MoveIt](https://moveit.picknik.ai/humble/index.html) robot manipulation software. The following chapter guides us to the steps taken and lessons learned that got me to this main result.

## How to Access my work


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


  This set-up was beneficial for me given my prior experience with CPP ROS2, allowing for fast learning due to familiar syntax. My idea is to     rewrite the pick and place tutorial for the case of our UR robot and use a ros1-bridge to communicate between the simulation and the motion     planner. I ran into the problem that [a ros1_bridge on ubuntu 22.04 is not straight-forward](https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html). It requires a.o. building ros and moveit from source (After a long time trying, I didn't manage to get all installations right). 
  
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

![Listener](https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExMzA5YjcyNjAxZWYyNGJmMDVjNmZiYTJiNGM0MmQ4MTQwN2IzODExYSZjdD1n/czI8dSwHJiKSdAVzn5/giphy.gif)


The ROS2 listener node prints in the terminal the name of one of the joints in the ROS1 Gazebo simulation. Hence, it is possible to use the rosbridge as intended! Unfortunately, another set-up problem arose while building on this idea. The pick and place task is highly relying on the Moveit Task Constructor, which appears to be unreleased for Moveit2 of ROS2 Foxy (it is only available for ROS2 Humble, which was eliminated already in step 2). I have at least not been able to resolve all dependency issues that I ran into, and this is where I dropped my contributions to make this test assignment work via a ROS2 solution. 

### 3) MoveIt - ROS1

MoveIt is also available for ROS1, which is syntax-wise more of a challenge to me, but another good [Pick and Place Tutorial for MoveIt](https://ros-planning.github.io/moveit_tutorials/doc/pick_place/pick_place_tutorial.html) turned out to be a good firestarter for my solution to the test assignment. I am now working on the following set-up:

| Ubuntu 20.04 | ROS1 NoEtic | Moveit |
|--------------|-------------|--------|

With this set-up I managed to complete the Tutorial in ROS1 by creating the `mtc_panda` package. The tutorial produces the following animation for pick and place motion with a Panda robot:

![Panda](https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExZmM2YzQyYzAyOTg3MGI2YzY1N2E3NGIzMmY1MzY2YWM4NDI5ZjEwOSZjdD1n/OR0xHe5doSbEYZm972/giphy.gif)

These results are reproducable by running the following sets of terminal commands in separate terminals:

T1:

```bash 
$ source ${ROS1_INSTALL_PATH}/setup.bash
$ source catkin_ws/devel/setup.bash
$ roslaunch mtc_panda demo.launch
```

T2:

```bash 
$ source ${ROS1_INSTALL_PATH}/setup.bash
$ source catkin_ws/devel/setup.bash
$ roslaunch mtc_panda demo.launch
```

Now the next step is to try and build upon this tutorial to solve the case. 

### 4) Generate a MoveIt ROS package via the moveit setup assistant and set-up the Gazebo link. 

I made use of the provided URDF file of the simulated robot to generate a MoveIt ROS package for the robot of interest. This generated `moveit_ur_robot` package required some tweeking, but allowed me to launch the UR Robot together with MoveIt into a RVIZ visualization. While doing this, I made another big step, since I managed to control the Gazebo Simulation by planning trajectorys from one robot configuration to another in the RVIZ visualization. This functionality can be visualized as follows:

![Panda](https://gifyu.com/image/SIr6S)

This animation can be reproduced by running the following sets of terminal commands in separate terminals:

T1:		

```bash 
$ source ${ROS1_INSTALL_PATH}/setup.bash
$ source catkin_ws/devel/setup.bash
$ roslaunch simple_scene gazebo.launch 
```

T2: 

```bash
source ${ROS1_INSTALL_PATH}/setup.bash
source catkin_ws/devel/setup.bash
roslaunch moveit_ur_robot demo_planning_execution.launch 
```      

What I learned from this step is that if I can solve the problem within the RVIZ environment, I should be able to use this link to the Gazebo simulation to finalize the assignment. 


### 5) Solve a Pick and Place task for the UR Robot with MoveIt. 

Building on the tutorial that was completed in step 3, the intention now is to pick and place the same object as was done previously with the Panda robot, but now with the UR Robot. I am proud to announce that this crucial step was completed succesfully. I have introduced the `mtc_ur_robot` package, which contains a pick and place task class in which the magic happens. The result was provided in the first section already, but for the sake of completeness, it is reported again here:

![PickAndPlace](https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExMmFmMGNjNDZmNDE1ODcyZjMwZjgwMTFhMjhlZjBkYzRkMThlMjcwNSZjdD1n/JZPQAZkFR3mmnkDC9Q/giphy.gif)

This animation can be reproduced by running the following sets of terminal commands in separate terminals:

T1:		

```bash 
$ source ${ROS1_INSTALL_PATH}/setup.bash
$ source catkin_ws/devel/setup.bash
$ roslaunch simple_scene gazebo.launch 
```

T2: 

```bash
$ source ${ROS1_INSTALL_PATH}/setup.bash
$ source catkin_ws/devel/setup.bash
$ roslaunch moveit_ur_robot demo_planning_execution.launch 
```     

T3: 

```bash
$ source ${ROS1_INSTALL_PATH}/setup.bash
$ source catkin_ws/devel/setup.bash
$ roslaunch mtc_ur_robot pickplace.launch 
```     

Taking a closer look at the implementation, it can be seen that the Pick and Place task was broken down into various stages. As such, a complex task can be simplified by solving the motion planning problems for the individual stages sequentially while guaranteeing state continuity on all respective state boundaries. The stages that are part of my solution can be observed in more detail in the following animation: 

![Stages](https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExN2RlN2M4NjUxYWJjNTM3Y2FiNmM5M2Y2OWMzY2I4ZjZiYzY1MjhjZSZjdD1n/kCEmTtRu8uEeTedopo/giphy.gif)

Stage definitions are intuitive and allow for easy extension of the pick and place motion (e.g. pick and place of multiple objects). It was therefore more important for me to be able to execute the planned trajectory in the Gazebo simulation. Unfortunately, I have been unsuccesful in forwarding the planned trajectory into the Gazebo simulation, though I am confident that I am not far of, given the results of step 3. 

### 6) Conclusion
In conclusion, I made huge progress on the test assignment and I am confident that this delivery can give you a good insight into my competences as a future robotics software engineer at REMY Robotics. The solution does show some limitations that I would be looking into as a next step: 

1) The motion planner tool sometimes reports the solution to be approximate, which would indicate small state incontinuities in the solution, and should be carefully observed before deploying such solutions on hardware. 

2) The end effector is currently implemented in a complex way including all joints that are in the relatively difficult part, and these are included in the motion planning. I think that this gripper should ideally be taken out of the motion planning for the manipulator arm as a whole (just open and close it at the right moments in time) and it would be a next step for me to try and see if that change improves the computational/numerical performance of the solution. 

Finally, I have to admit that this deliverable cost me significantly more time than anticipated. I had a great learning curve along the way, worked hard for two full weeks on the test and am proud of what is in this repository today. I am looking forward to your response and insights into my submission. 

## Evaluation

You can use any 3rd party package. We would like to see how you design the system and the planning & control pipeline.

## Submission

A docker image and the source code are required. Please explain your design choices. If it is not possible due to circumstances, please send us source codes with clear instructions and explanations.

## Additional Questions

- How would you improve the efficiency and reliability?
- What would you do if we needed multiple robots in a system?
- How would you deploy the application to multiple physical locations? What is needed to scale it?

