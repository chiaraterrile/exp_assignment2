# exp_assignment2

### Introduction
In this project a dog wheeled robot is able to move in different positions and if detects the presence of a green ball he follows it. This robot can assume three behaviours which are **sleep**,**normal** and **play**. The robot acts in simulation on Gazebo in an arena represented as an 8x8 meters grid containing the robot, the human and a green ball.

### Software architecture
The architecture is based on three main blocks. The first one is given by the user interface, where the user is able to control the movement and the position of a green ball. Whenever the z-coordinate of the ball is given positive, this is interpreted as a command *play* given by the user to the robot, so a message of start is published to the topic */ball/chatter*. The second block is a state machine that subscribes to the topic /ball/chatter to estabilish in which state to go. If tha ball is in the arena the robot will go in the **play** state, if not it will go in the **normal** or **sleep** in a random way.
From the state machine, in the states normal and sleep, an action client send the goal position to an action server (the third block) that makes the robot move in the desired position, that can be the home position or a random position.
In case of the play state, once the robot reaches the ball, a message is published to the topic /robot/joint1_position_controller/command which controls the joint of the head of the robot in order to make it rotate of 45° left and right passing through the center.
### Packages and file list
The package used is **exp_assignment2**.
In the folder `exp_assignment2/scripts` there are the following files :
- **go_to_point_ball.py** : this is an action server that moves the ball in the desired position and publishes a message in case the ball is in the arena 
- **go_to_point_action.py** : this is an action server that moves the robot in the desired position 
- **state_machine.py** that is the FSM that controls the behaviour of the robot in the three cases.

In the folder `exp_assignment2/urdf` there are all the files describing the structure of the robot, of the ball and of the human present in the scene. ( robot.xacro, robot.gazebo, human.urdf, ball.xacro, ball.gazebo)

In the folder `exp_assignment2/launch` there's the launch file **gazebo_world.launch** that launches the simulation in Gazebo.

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
One of the limitations of the system is that when the user puts the ball in a certain position, he has to wait that the robot exits from the state where it is before it starts interacting with the ball.

### Possible technical improvements
A possible technical improvement is to manage the situation described before giving a major priority to the action of positioning the ball in the arena with respect to the execution of the other states.

### Author and contact
Terrile Chiara
mail: **chiaraterrile97@gmail.com**
