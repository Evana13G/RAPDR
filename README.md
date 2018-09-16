********************************************************
**                  Run instructions                  **
********************************************************

STEP 1: This just spawns the baxter. 
>> roslaunch baxter_gazebo baxter_world.launch


STEP 2: This one spawns the models and creates publishers for the 2 buttons, the block behind the wall, and the wall. For some reason, the position of the buttons is shown to be at the right bottom corner, which might be an easy fix from the URDF model. This program runs forever as it is publishing the position of the objects.
>> rosrun search_and_rescue_sim search_and_rescue.py

STEP 3: This node tells the wall to either be raised or lowered. By default it is raised, and then whenever a boolean false is published to a certain topic, it lowers the wall. So I was thinking that I could create a publisher inside the file that moves the baxter joints so that it publishes the location of the grippers, and whenever either of the grippers is below a certain distance threshold from the buttons, a boolean false is published to that node so that the wall is lowered.Then as soon as the gripper is not close to the button anymore, the boolean goes back to true so that the wall is raised. 
>> rosrun action_primitive_variation raise_wall

STEP 4: This program is inside the action_primitive_variation package and it was my attempt at getting the grippers to "press" (just hover at a certain distance) the buttons. I haven' gotten it to work properly yet. It is also just a modified version of the ik_position baxter example script. 
>> rosrun action_primitive_variation raise_wall.py 

STEP 5: And finally, if you want to control the wall as in the video, you can write a publisher on the command line like this: 
>> rostopic pub /testSub std_msgs/Bool "data: true" -1 (or data:false if you want to lower the wall)