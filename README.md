# Robotics Software Engineer Task

I was asked to design and implement a pick-and-place application using a Gazebo simulation environment. The expected result of this task is a docker image and a GitHub repo with the source code.

## Main Result

The  main result of my work on this Robotics Software Engineer task is summarized in the following animation. 

[<img src="https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExMmFmMGNjNDZmNDE1ODcyZjMwZjgwMTFhMjhlZjBkYzRkMThlMjcwNSZjdD1n/JZPQAZkFR3mmnkDC9Q/giphy.gif" width="75%"/>](https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExMmFmMGNjNDZmNDE1ODcyZjMwZjgwMTFhMjhlZjBkYzRkMThlMjcwNSZjdD1n/JZPQAZkFR3mmnkDC9Q/giphy.gif)

A motion planning methodology was developed by making use of the [MoveIt](https://moveit.picknik.ai/humble/index.html) robot manipulation software. The following chapters guide you through the steps taken and lessons learned that got me to this main result. Also, it allows you to assess my technical competences for the position of Robotics software engineer at REMY Robotics.

## How to Access my work

I have tried to include my work into the provided docker image, but I ran into some conflicting dependencies issues. Therefore, I propose to build my solution from source via the following steps. These build instructions assume the `${ROS1_INSTALL_PATH}` (ROS1 NoEtic, e.g. `/opt/ros/noetic`) variable and the ${RMW_IMPLEMENTATION} (e.g. rmw_cyclonedds_cpp) variables to be set in your `.bashrc`file. Please check whether rosdep installed all required dependencies. Also, it is assumed that MoveIt (including the Moveit Task Constructor package) is available into the `~/ws_moveit` workspace. It should then be possible to succesfully build my solutions on Ubuntu 20.04 via the following set of terminal commands:

T1:

```bash
$ cd catkin_ws
$ source ${ROS1_INSTALL_PATH}/devel/setup.bash
$ source ~/ws_moveit/devel/setup.bash
$ catkin build
$ source devel/setup.bash
```
Please let me know in case any problems rise while trying to compile my solution. I am very open to give the docker route another shot in that case. Also I would be open for a quick call to guide you through the compilation process.

## Steps Taken

### 1) Starting situation

  The starting situation can be viewed via the following terminal commands:

  ```bash 
  $ source ${ROS1_INSTALL_PATH}/setup.bash 
  $ source catkin_ws/devel/setup.bash
  $ roslaunch simple_scene gazebo.launch
  ```
  
  In short, a Gazebo simulation of a manipulator arm is provided and the objective is to move all blocks from one table to the other. An overview can be seen in the following animation:


[<img src="https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExMjIwN2IwOTI4ZmM1ZjI3NWIxMmFmYjI4MTE2OWVkODhkZGUyNGM4MiZjdD1n/EIBFqhjWdSIOTZbTUA/giphy.gif" width="75%"/>](https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExMjIwN2IwOTI4ZmM1ZjI3NWIxMmFmYjI4MTE2OWVkODhkZGUyNGM4MiZjdD1n/EIBFqhjWdSIOTZbTUA/giphy.gif)

### 2) Moveit2 - Ubuntu 22.04

  A literature review and my notes from the previous talks with REMY robotics pointed me at the MoveIt manipulator software that can be used for solving motion planning problems to robotic manipulators. I found a [pick and place with moveit task constructor tutorial](https://moveit.picknik.ai/humble/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html), which obviously can be a great starting point in solving this test assignment. I followed the tutorial succesfully (files allowing for reproducability are in the  `mtc_tutorial` package of the `colcon_ws` ROS2 Humble workspace) with the following set-up:

| Ubuntu 22.04 | ROS2 Humble | ROS1 NoEtic | Moveit2 |
|--------------|-------------|-------------|---------|



This set-up was beneficial for me given my prior experience with CPP ROS2, allowing for fast learning due to familiar syntax. My idea is to     rewrite the pick and place tutorial for the case of our UR robot and use a ros1-bridge to communicate between the simulation and the motion     planner. I ran into the problem that [a ros1_bridge on ubuntu 22.04 is not straight-forward](https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html). It requires a.o. building ros and moveit from source (After a long time trying, I didn't manage to get all installations and dependencies right). 
  
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


The ROS2 listener node prints in the terminal the name of one of the joints in the ROS1 Gazebo simulation. Hence, it is possible to use the rosbridge as intended! Unfortunately, another set-up problem arose while building on this idea. The pick and place task is highly relying on the Moveit Task Constructor, which appears to be unreleased for Moveit2 in ROS2 Foxy (it is only available for ROS2 Humble, which was eliminated already in step 2). I have at least not been able to resolve all dependency issues that I ran into, and this is where I dropped my contributions to make this test assignment work via a ROS2 solution. Nevertheless, these insights can be of value while implementing REMY robotics desired upgrade from ROS1 to ROS2, and are therefore included in this report. A recommended next step here could be to run the individual applications each in their own docker container to resolve the OS dependency issues that I dealt with, but I have been unsuccesfull in bridging topics out of a container and therefore decided to leave this as is.

### 3) MoveIt - ROS1

MoveIt is also available for ROS1, which is syntax-wise more of a challenge to me, but another good [Pick and Place Tutorial for MoveIt](https://ros-planning.github.io/moveit_tutorials/doc/pick_place/pick_place_tutorial.html) turned out to be a good firestarter for my solution to the test assignment. I am now working on the following set-up:

| Ubuntu 20.04 | ROS1 NoEtic | Moveit |
|--------------|-------------|--------|

With this set-up I managed to complete the Tutorial in ROS1 by creating the `mtc_panda` package. The tutorial produces the following animation for pick and place motion with a Panda robot:

[<img src="https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExZmM2YzQyYzAyOTg3MGI2YzY1N2E3NGIzMmY1MzY2YWM4NDI5ZjEwOSZjdD1n/OR0xHe5doSbEYZm972/giphy.gif" width="75%"/>](https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExZmM2YzQyYzAyOTg3MGI2YzY1N2E3NGIzMmY1MzY2YWM4NDI5ZjEwOSZjdD1n/OR0xHe5doSbEYZm972/giphy.gif)

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

I made use of the provided URDF file of the simulated robot to generate a MoveIt ROS package for the robot of interest. This generated `moveit_ur_robot` package required some tweaking, but allowed me to launch the UR Robot together with MoveIt into a RVIZ visualization. While doing this, I made another big step, since I managed to control the Gazebo Simulation by planning trajectorys from one robot configuration to another in the RVIZ visualization. This functionality can be visualized as follows:

[<img src="https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExMTEzN2FjYjVkYjhkN2QzYjk0YzQ3NWY5YzExNzA2YWI1YTJmODAwZSZjdD1n/bXI8ubUrVKrzxFN7Rw/giphy.gif" width="75%"/>](https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExMTEzN2FjYjVkYjhkN2QzYjk0YzQ3NWY5YzExNzA2YWI1YTJmODAwZSZjdD1n/bXI8ubUrVKrzxFN7Rw/giphy.gif)

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

What I learned from this step is that if I can solve the problem within the RVIZ environment, I should be able to use the interface that is established in this step to deploy the motion planning to the Gazebo simulation to finalize the assignment. 


### 5) Solve a Pick and Place task for the UR Robot with MoveIt. 

Building on the tutorial that was completed in step 3, the intention now is to pick and place the same object as was done previously with the Panda robot, but now with the UR Robot. I am proud to announce that this crucial step was completed succesfully. I have introduced the `mtc_ur_robot` package, which contains a pick and place task class in which the magic happens. The result was provided in the first section already, but for the sake of completeness, it is reported again here:

[<img src="https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExMmFmMGNjNDZmNDE1ODcyZjMwZjgwMTFhMjhlZjBkYzRkMThlMjcwNSZjdD1n/JZPQAZkFR3mmnkDC9Q/giphy.gif" width="75%"/>](https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExMmFmMGNjNDZmNDE1ODcyZjMwZjgwMTFhMjhlZjBkYzRkMThlMjcwNSZjdD1n/JZPQAZkFR3mmnkDC9Q/giphy.gif)

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

Taking a closer look at the implementation, it can be seen that the Pick and Place task was broken down into various stages. As such, a complex task can be simplified by solving the motion planning problems for the individual stages sequentially while guaranteeing state continuity on all respective stage boundaries. The stages that are part of my solution can be observed in more detail in the following animation: 

[<img src="https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExN2RlN2M4NjUxYWJjNTM3Y2FiNmM5M2Y2OWMzY2I4ZjZiYzY1MjhjZSZjdD1n/kCEmTtRu8uEeTedopo/giphy.gif" width="75%"/>](https://media.giphy.com/media/v1.Y2lkPTc5MGI3NjExN2RlN2M4NjUxYWJjNTM3Y2FiNmM5M2Y2OWMzY2I4ZjZiYzY1MjhjZSZjdD1n/kCEmTtRu8uEeTedopo/giphy.gif)

Stage definitions are intuitive and allow for easy extension of the pick and place motion (e.g. pick and place of multiple objects). It was therefore more important for me to be able to execute the planned trajectory in the Gazebo simulation. Unfortunately, I have been unsuccesful in forwarding the planned trajectory into the Gazebo simulation, though I am confident that I am not far of, given the results of step 4. 

### 6) Conclusion
In conclusion, I made huge progress on the test assignment and I am confident that this delivery can give you a good insight into my competences as a future robotics software engineer at REMY Robotics. The solution does show some limitations that I would be looking into as a next step: 

1) The motion planner tool sometimes reports the solution to be approximate, which would indicate small state incontinuities in the solution, and should be carefully observed before deploying such solutions on hardware. 

2) The end effector is currently implemented in a complex way including all joints that are in the relatively difficult part, and these are included in the motion planning. I think that this gripper should ideally be taken out of the motion planning for the manipulator arm as a whole (just open and close it at the right moments in time) and it would be a next step for me to try and see if that change improves the computational/numerical performance of the solution. 

3) I have seen motion planning attempts in which the object is approached from behind (in negative x-direction of the end effector frame) before picking, causing the manipulator hand to move through the object rather than approaching it by moving along the positive x-axis of the end effector frame. This behavior seems dependent on the initial state of the robot and the object location and requires therefore a more elaborate analysis. 

Finally, I have to admit that this deliverable cost me significantly more time than anticipated. I had a great learning curve along the way, worked hard for two full weeks on the test and am proud of what is in this repository today. I am looking forward to your response and insights into my submission. 


## Additional Questions

### 1) How would you improve the efficiency and reliability?

Improvement points in terms of efficiency and reliability for my current solution to the pick and place task for the UR robot are listed already in the conclusion section above. Moreover, in terms of reliability, I would recommend to implement proper unit testing of the developments such that limitations can be pinpointed more easily and our confidence in proper working of each part of the code increases. As a first step, all (sub)stages of the motion planning task are already subdivided into separate functions, but a lack of time did not allow me to implement proper testing yet. In a more practical sense, I would be coupling the pick and place task to the application of robotic kitchens and note that an important constraint would be that the object may never be tilted while being transported to prevent food waiste and ensure a clean kitchen. Also, the transported object may contain liquids that could be spilled under relatively accelerations. A more detailed model of the dish dynamics could be deployed to predict and prevent such challenging situations. I also heard a podcast/talk from the REMY Robotic' CEO explaining challenging situations in which e.g. a fly ends up in the dish under transport. These kind of corner cases should be detectable and the motion planning task should be able to deal with this appropriately; such contaminated dishes being delivered to customers can severly damage the companies repuattion. While trying to further improve the motion planning efficiency, I would at first have a look at the specific motion planning algorithm that is invoked by MoveIt and see whether a less advanced method would be able to solve our case also. 

### 2) What would you do if we needed multiple robots in a system?

I interpret this question as "What opportunities can you think of for applying multiple robots in a single robotic kitchen?". Obviously, the simplest application of a second robot is to basically build a second parallel robotic kitchen stack to improve the total kitchen capacity by copying a complete robotic kitchen including manipulator to a nearby location. On top of that, one can think of how the manipulators can collaborate while cooking. For example, in case one robot is idle and the other is cooking, the idle robot can verify whether it can do some of the upstream work that the cooking robot has not been able to start with (if it can reach the right ingredients and drop-off locations). Another opportunity I see for scaling the robotic kitchens under the assumption of multiple robots being present, is to simplify the 'smart ovens' and 'smart freezers', as one idle robot can open the oven's door for the other robot that is bringing the food. You have basically introduced the cook's second arm and should therefore be able to at least match all physical actions that a human cook is able to do with both hands. The application could even be extended to mise-en-place, as one manipulator can hold the product while the other cuts it, etc. The possible extensions are virtually unlimited and I would be super excited to be a part of it.  

### 3) How would you deploy the application to multiple physical locations? What is needed to scale it?

When deploying the application to multiple physical locations, one would need to set-up a supervisory data collection and monitoring system that gathers important successes and failures from the robotic kitchens in various branch locations. As such, the technologies that drive the individual branches can learn from experiences in other branches to improve the overall system quality more quickly. In order to scale the technology, I assume that step-in costs for potential customers should be a main focus point. I expect the technology to be relatively expensive still for especially smaller restaurants that are not afiliated to a large branch. I would be trying to answer the question as to what are the main cost drivers in this technology and how can we find the right balance between system performance and affordability for an optimal market position. 

