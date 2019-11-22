# Vision and Robotics
This project is about Robot Vision and Robot control with a non-planar (3D) robot with 4 degrees of freedom with two horizontal cameras on yz and xz axis.
From the target cameras data, joint state estimation, target dection, Forward Kinematics, and Closed-Loop Contorl are acheived.
In package ivr_assignment we uses three ROS nodes namely, image1.py, image2.py, RobotController.py, where that data collected and preprocessed by two images processng nodes are sychronized are computed and used to control the robot.
 
#Installation and running this project

*Clone the repo then build using
 catkin_make
 
*make sure python scripts at are executable by running
 
 chmod +x /src/ivr_assignment/src/*.py
 
*Launch the library 
 source devel/setup.bash
 roslaunch ivr assignment spawn.launch
  
