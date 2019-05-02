## Description

* A python program that interfaces with the Virtual Robot Experimentation platform (V-REP) and controls the movements of a robot arm.

* It also receives camera images and sensor readings from the robot arm.
* A neural network processes the input data to produce actuation in the Robot's joints so that the end-effector can reach a target position without the use of inverse kinematics.
* The neural network learns about the robot configuration, and movement through reinforcement learning.

## In the main() function there are 3 things possible.

### You can un-comment any one.

1. run the random agent (nicely shows, the simulation and arm movement)
2. stream videos from arm (this streams the camera images from the 2 cameras on the arm)
3. RL-agent (Uses neural network (DQL) to control arm) [still under development, the target function for neural network is still not implemented]
