# robotics_project

## To run the environments
Firstly we need to open 4 new terminals.

In T1 run the following command to open CoppeliaSim:

` ~/apps/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04/coppeliaSim.sh`

In T2 run the following commands to build the project:

`cd /dev_ws`

`colcon build`

In T3 run the following commands to bridge:

`cd /dev_ws/src`

ros2 launch robomaster_ros s1.launch name:=RM0001 tof_0:=true tof_1:=true tof_2:=true tof_3:=true


You can get the name of the robomaster inside Coppelia by double-clicking as shown in the first image and copy the third parameter of the function `simRobomaster.create_s1()` as the second figure shows.

<img width="516" alt="Screenshot 2023-05-12 at 10 45 06" src="https://github.com/JuryAndrea/robotics_project/assets/43291397/1e1b5508-145b-4bea-a5f1-bad3b0a6e683">


<img width="596" alt="Screenshot 2023-05-12 at 10 46 15" src="https://github.com/JuryAndrea/robotics_project/assets/43291397/61b20321-2949-4c10-9a7c-e068711b397b">

In T4 run the following commands the launch the xml file:

`cd /dev_ws/src`

`source ~/dev_ws/install/setup.bash`

`ros2 run robomaster_proj robomaster_node`

Optionally, we can open a new terminal T5 in order to show the topic list of the robomaster. To do that:

`cd /dev_ws/src`

`ros2 topic list`

Note that you have to bridge and start the simulation before you can show the topic list.

You can source T3 and T4 just once. 
