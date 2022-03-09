# Kinematics of Scara Robot
## *RBE 500: Foundations of Robotics - Worcester Polytechnic Institute, Fall 2021*

## Requirements:

1. Ubuntu with ROS Noetic
2. Gazebo Simulator

## How to run the code?

First clone the repository into a `src` folder of a ROS workspace and perform catkin_make operation. Use the source command:
  ```
  source devel/setup.bash
  ```

> IMPORTANT: You will need to use this command in every new terminal you open, or you can add it to your .bashrc file

1. ### Using a URDF to spawn the SCARA robot
    To spawn the scara robot in ROS-Gazebo environment, use the roslaunch command: 
    ```
    roslaunch scara_robot my_env.launch
    ```

2. ### Forward Kinematics Implementation
    #### (Subscriber - Publisher model)
    The forward kinematics node subscribes to the topic */scara_robot/joint_states* through which it receives the joint values of robot from gazebo. Then these three joint values are used to calculate the end effector pose using forward kinematics and then is published on the topic */scara_robot/output_pose*.

    In a new terminal run the command to run the forward kinematics node:
    ```
    rosrun scara_robot scara_forward
    ```

    ---
    In a new terminal run the following command to observe the Pose of the End Effector (output from the forward kinematics node):
    ```
    rostopic echo /scara_robot/output_pose
    ```

    In the output, we only observe the end effector position, that is, the x, y and z values.

    ---
    In another terminal run the command to publish new values to the joints:

    ```
    rostopic pub /scara_robot/joint1_position_controller/command std_msgs/Float64 "data: 0.78"
    ```
    Here, we can change the controller from joint1_position_controller to joint2_position_controller.

    ---
3. ### Inverse Kinematics Implementation
    #### (Service Client model)
    The server file takes the pose values using geometry_msgs/Pose message type. Then server takes the position x, y and z values from Pose.position object and calculates the three joint variables joint1, joint2 and joint3 using inverse kinematics. Then the server file prints the joint state response using sensor_msgs/JointState message type. 

    In another terminal run the command to publish new values to the joints (with the forward kinematics running):

    ```
    rosservice call /inv
    ```
    and then use tab completion to get the standard message format, in which you can enter end effector positions
