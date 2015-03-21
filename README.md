### youbot_499 

Required Packages
------------------------
To run this on Gazebo, you HAVE to install the youbot gazebo* interface as per instructions in [this link](http://www.youbot-store.com/wiki/index.php?title=Gazebo_simulation&hmswSSOID=10b4d7be36c130126e02a9c81ce579a7f71c954f)
Otherwise, every other dependency is usually available on a standard ROS Installation.

Launch File 
------------------------
To launch the demo application of the youbot moving around an object on Gazebo, simply run

`roslaunch youbot_499 youbot_gazebo_circle.launch`

This pops up a rqt_reconfigure window. Choose youbot_circle and Set values for commanded distance from object and PID gains for both position and orientation controller. Set a linear Y velocity and uncheck the emergency stop (estop). Sample values are :
* refd = 1.0
* linear_y = 0.05 / 0.1
* kp (position) = -2.0
* kpp (orientation) = -2.0

There is also pointless application that lets you move the youbot around using rqt_reconfigure. To launch it on Gazebo, simple run

`roslaunch youbot_499 youbot_gazebo_move.launch`

This again pops up a rqt_reconfigure window which lets you control the velocity of the youbot.

Nodes
------------------------
This is the real essence of this repository. There are currently the following nodes in this repository: 
1. `youbot_vel_dynamic` which lets you control the base velocities through dynamically reconfigurable parameters.      
2. `youbot_circle` which lets you move in a circle around an object. The controller gain's, distance to object and emergecy stop are all dynamically reconfigurable parameters.
3. 'lscan_angle_filter' this node lets you dynamically filter the laser scanner output by setting ranges of hits, beyond the required angle of view, very HIGH. 
4. `cropbox_dynamic` reads the current distance to the object (which, due to our controller is the same as the commanded distance), reads the current tf tree and outputs an estimate of the position of the object in the ASUS's frame. 
5. `cropbox_control` reads the output from the previous node and dynamically sets the cropbox filters x y and z ranges.

Note: Node 5 (cropbox_control) is not fully functional yet, because I havent yet tried it on the youbot. It requires an ASUS connected to the system with a nodelet running, which is currently not the case in the Gazebo demos.


To do
-----------------------
1. Initialise arm to a particular position
2. Add launch file to run it on the youbot with a cropbox nodelet running

