################ Prerequisites ################ 
(Download in this order)

1. Ubuntu 16.04
2. ROS Kinetic (http://wiki.ros.org/kinetic/Installation/Ubuntu & http://wiki.ros.org/ROS/Tutorials)
3. Gazebo 7 (Installs with ROS installation) 
4. Baxter Gazebo Simulation (http://sdk.rethinkrobotics.com/wiki/Simulator_Installation)
5. ROSPlan (https://github.com/KCL-Planning/ROSPlan/wiki) (subject to change)

The file heirarchy should take the following form:
catkin_ws
---src
------RAPDR
------*all baxter gazebo packages*
------rosplan

################ Install Instructions ################ 
Pull all of the code above and make sure you do the following:
>> cd ~/catkin_ws (navigate yourself to the catkin_ws level)
>> source devel/setup.bash (add this line to your .bashrc file if you want to source this permanently)
>> catkin_make

############### Run instructions ############### 
Each of the following should be run in a separate terminal window:

STEP 1: This just spawns the baxter. 
>> roslaunch baxter_gazebo baxter_world.launch

STEP 2: This one spawns the models and creates publishers for the 2 buttons, the block behind the wall, and the wall. For some reason, the position of the buttons is shown to be at the right bottom corner, which might be an easy fix from the URDF model. This program runs forever as it is publishing the position of the objects.
>> rosrun initialize initialize.py

STEP 3: This node tells the wall to either be raised or lowered. By default it is raised, and then whenever a boolean false is published to a certain topic, it lowers the wall. So I was thinking that I could create a publisher inside the file that moves the baxter joints so that it publishes the location of the grippers, and whenever either of the grippers is below a certain distance threshold from the buttons, a boolean false is published to that node so that the wall is lowered.Then as soon as the gripper is not close to the button anymore, the boolean goes back to true so that the wall is raised. 
>> rosrun action_primitive_variation raise_wall

This program is inside the action_primitive_variation package and it was my attempt at getting the grippers to "press" (just hover at a certain distance) the buttons. I haven' gotten it to work properly yet. It is also just a modified version of the ik_position baxter example script. 
>> rosrun action_primitive_variation raise_wall.py 

If you want to control the wall as in the video, you can write a publisher on the command line like this: 
>> rostopic pub /testSub std_msgs/Bool "data: true" -1 (or data:false if you want to lower the wall)

STEP 4: This is an example of how you get the action servers to run. All server files are located in RAPDR/action_primitive_variation/scripts. Each one has to run in its own terminal. We will eventually move these all into a launch file
>> rosrun action_primitive_variation close_gripper.py

STEP 5: To execute a full run of the pick and place scenario....

STEP 6: To run the PDDL planner ....

################ Other Info ################ 
The URDF models are inside the baxter_simulation package in a folder that I believe is called baxter_sim_examples/models. The URDF model for the table and the wall is called cafe_table. 
