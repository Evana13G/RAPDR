#### Setup Instructions
Setup each of the following, in order:

1. Ubuntu 16.04

2. ROS Kinetic (http://wiki.ros.org/kinetic/Installation/Ubuntu & http://wiki.ros.org/ROS/Tutorials)

3. Gazebo 7 (Installs with ROS installation) 

4. Baxter Gazebo Simulation (http://sdk.rethinkrobotics.com/wiki/Simulator_Installation)

    1. Note, we are not using the `baxter.sh sim` script, but you may use that during initial setup, as described in their instructions.

5. ROSPlan (https://github.com/KCL-Planning/ROSPlan/wiki) (subject to change)
   1. Note the download and installation instructions are located at
      https://github.com/KCL-Planning/ROSPlan
   2. The instructions neglect to mention installing prerequisite:
      `sudo apt-get -y install bison`
   3. The installation instructions would have you build using `catkin build`, but we are using a `catkin_make` project, so build as follows:

            cd ~/catkin_ws
            catkin_make
            source devel/setup.bash

6. Clone the RAPDR project:

        cd ~/catkin_ws/src
        git clone git@github.com:Evana13G/RAPDR.git
          (or git clone https://github.com/Evana13G/RAPDR.git)

7. The file heirarchy should take the following form:

        catkin_ws/
        catkin_ws/src/
        catkin_ws/src/RAPDR/
        catkin_ws/src/*all baxter gazebo packages*
        catkin_ws/src/rosplan/

8. Build:

        cd ~/catkin_ws
        catkin_make
        source devel/setup.bash

#### Run instructions
Each of the following should be run in a separate terminal window:

1. This just spawns baxter. 

        cd ~/catkin_ws && source devel/setup.bash && roslaunch baxter_gazebo baxter_world.launch

2. This one spawns the models and creates publishers for the 2 buttons, the block behind the wall, and the wall. For some reason, the position of the buttons is shown to be at the right bottom corner, which might be an easy fix from the URDF model. This program runs forever as it is publishing the position of the objects.

        cd ~/catkin_ws && source devel/setup.bash && rosrun initialize_environment initialize_environment.py

3. Both of these services must be running.

    1. Service for obtaining object

            cd ~/catkin_ws && source devel/setup.bash && rosrun agent obtain_object.py

    2. Service for pressing button

            cd ~/catkin_ws && source devel/setup.bash && rosrun agent press_button.py

4. Now actually test. You should see Baxter actually grasp the object:

        cd ~/catkin_ws && source devel/setup.bash && rosrun agent testServer.py

5. Everything below is not yet tested. Please update.

6. This node tells the wall to either be raised or lowered. By default it is raised, and then whenever a boolean false is published to a certain topic, it lowers the wall. So I was thinking that I could create a publisher inside the file that moves the baxter joints so that it publishes the location of the grippers, and whenever either of the grippers is below a certain distance threshold from the buttons, a boolean false is published to that node so that the wall is lowered.Then as soon as the gripper is not close to the button anymore, the boolean goes back to true so that the wall is raised. 

        cd ~/catkin_ws && source devel/setup.bash
        rosrun action_primitive_variation raise_wall.py

7. This program is inside the action_primitive_variation package and it was my attempt at getting the grippers to "press" (just hover at a certain distance) the buttons. I haven' gotten it to work properly yet. It is also just a modified version of the ik_position baxter example script. 

        rosrun action_primitive_variation raise_wall.py 

8. If you want to control the wall as in the video, you can write a publisher on the command line like this: 

        rostopic pub /testSub std_msgs/Bool "data: true" -1 (or data:false if you want to lower the wall)

9. This is an example of how you get the action servers to run. All server files are located in RAPDR/action_primitive_variation/scripts. Each one has to run in its own terminal. We will eventually move these all into a launch file

        rosrun action_primitive_variation close_gripper.py

10. To execute a full run of the pick and place scenario....

11. To run the PDDL planner ....

#### Other Info
The URDF models are inside the baxter_simulation package in a folder that I believe is called baxter_sim_examples/models. The URDF model for the table and the wall is called cafe_table. 

1. For proof of concept scenario #1, we assume the following protocol:

"left" - left gripper
"right" - right gripper
"block" - object to obtain
"left_button" - left button
"right_button" - right button

 