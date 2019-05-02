Go to [Documents/RL Work/VREP-ARM-Interface]
and open arm-controll.py
run by ctrl+b

In the main() function there are 3 things possible. 

You can uncomment any one.

1 - run the random agent (nicely shows, the simulatio and arm movement)
2 - stream videos from arm (this streams the camera images from the 2 cameras on the arm)
3 - RL-agent (Uses nueral network (DQL) to control arm) [still under development, the target function for nueral network is still not implemented]