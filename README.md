# Robotics Final Project 
##### Group Z: Altrichter Christian, Jury D'Onofrio and Francesco Huber
GitHub repository: https://github.com/Altricch/robomaster_proj.git

## To run the environments
Firstly we need to open 4 new terminals.

In T1 run the following command to open CoppeliaSim:

` ~/apps/CoppeliaSim_Edu_V4_4_0_rev0_Ubuntu20_04/coppeliaSim.sh`

In T2 run the following commands to build the project:

`cd /dev_ws`

`colcon build`

In T3 run the following commands to bridge:

`cd /dev_ws/src`

`ros2 launch robomaster_ros s1.launch name:=RM0001 tof_0:=true tof_1:=true tof_2:=true tof_3:=true`

In T4 run the following commands the launch the xml file:

`cd /dev_ws/src`

`source ~/dev_ws/install/setup.bash`

`ros2 run robomaster_proj robomapper_node2`

You can source T3 and T4 just once. 

## Scene Selection
In order to test the robot we suggest the following scenes:
- Base_Scene.ttt (Basic square room with angular wall section in top left corner)
- Base_Scene2.ttt (Same as previous but changed starting position)
- Base_Scene_Plant.ttt (Same as original stress test but with obstacles)
- Stress_Test.ttt (Version of previous scene with unreachable bottom left corner)
- Stress_Test2.ttt (Square room with big diagonal section)
- Stress_Test3.ttt (Large Rectangular Room with some abstacles)
- Stress_Test4.ttt (Same as stress test 2 with changed starting position)
- Stress_Test5.ttt (Large Room with irregular walls)
- Stress_Test6.ttt (Same as stress test 5 with changed starting position)



