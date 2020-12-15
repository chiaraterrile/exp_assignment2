# exp_assignment2

### Introduction
In this project a dog wheeled robot is able to move in different positions and if detects the presence of a green ball he follows it. This robot can assume three behaviours which are **sleep**,**normal** and **play**. The robot acts in simulation on Gazebo in an arena represented as an 8x8 meters grid containing the robot, the human and a green ball.

### Software architecture
The architecture is based on three main blocks. The first one is given by the user interface, where the user is able to control the movement and the position of a green ball. The second block is basically a state machine where the robot goes by default in the **normal** state (RANDOMLYGOING), and it reaches a random position. Then it goes in the **sleep** state where it reaches a fixed home position passing at the end in the **play** state. The robot remains in this state until he's able to detect the ball, if he doesn't detect it for an amount of time returns to the normal state. Until the ball is detected the robot keeps moving the head of 45° left and right, passing through the center. 
From the state machine, in the states normal and sleep, an action client send the goal position to an action server (the third block) that makes the robot move in the desired position, that can be the home position or a random position.
### Packages and file list
The package used is **exp_assignment2**.
In the folder `exp_assignment2/scripts` there are the following files :
- **go_to_point_ball.py** : this is an action server that moves the ball in the desired position 
- **go_to_point_action.py** : this is an action server that moves the robot in the desired position 
- **state_machine.py** that is the FSM that controls the behaviour of the robot in the three cases.

In the folder `exp_assignment2/urdf` there are all the files describing the structure of the robot, of the ball and of the human present in the scene. ( robot.xacro, robot.gazebo, human.urdf, ball.xacro, ball.gazebo)

In the folder `exp_assignment2/launch` there the launch file **gazebo_world.launch** that launches the simulation in Gazebo.

In the folder `exp_assignment2/action` there's the file Planning.action which is the file containing the message for the two action servers.

### Installation and running procedure
To run the simulation it's necessary to put the package in a ROS workspace and then in the terminal run:
```
$ catkin_make
```
Then to launch the simulation:
```
$ roslaunch exp_assignment2 gazebo_world.launch
```
In another terminal to run the state machine :
```
$ rosrun exp_assignment2 state_machine.py
```
Whenever the user want to make the robot interact with the ball in another terminal has to run :
```
$ rostopic pub /reaching_goal/goal....
```
in order to send the message to the action server of the ball and position it in the desired position. To make the ball disappear the user has simply to set the z-coordinate of the position to a negative value.


### System's limitations
One of the limitations of the system is that when the user puts the ball in a certain position, he has to wait that the robot wakes up from the sleeping state before it starts interacting with the ball.

### Possible technical improvements
A possible technical improvement is to manage the situation described before using a flag to make the robot change the state whenever the user moves the ball.

### Author and contact
Terrile Chiara
mail: **chiaraterrile97@gmail.com**
