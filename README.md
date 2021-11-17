# Development of Scara Robot

This project involves:
1. Creating Xacro file (similar to URDF) for a planar SCARA robot, which has 2 revolute and 1 prismatic joint.
2. Creating position controllers for conrolling the robot
3. Creating forward kinematics ROS node, which will take input of the joint angles and return the corresponding end effector pose
4. Creating a inverse kinematics ROS node, which will take input of the end effector pose and return the required joint angle values in order to achieve the input pose
