#!/usr/bin/env python3 	

from scara_robot.srv import jacob
from scara_robot.srv import inv_jacob
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import rospy
import numpy as np
from math import sin, cos

# a0 + a1 are the lengths of the two segments of link1, 
# a2 is the length of link2
# h0 is the base cylinder offset
a1 = 1
a2 = 1
a0 = 1
h0 = 0.3

J = []

def calculate_A(theta, d, a, alpha):
    c = cos(theta)
    s = sin(theta)
    t = cos(alpha)
    y = sin(alpha)
    t1 =[[c,-s*t,s*y,a*c],[s,c*t,-c*y,a*s],[0,y,t,d],[0,0,0,1]]
    return t1

def get_Jacob(data):
    # Callback function for subscriber, returns joint states for calculating Jacobian
    global J
    q1 = data.position[0]
    q2 = data.position[1]
    d3 = data.position[2]

    A1 = calculate_A(q1, a0+h0, a1, 0)
    A2 = calculate_A(q2, 0, a2, np.pi)
    A3 = calculate_A(0, d3, 0, 0)

    H10 = np.array(A1)
    H20 = np.matmul(A1, A2)
    H30 = np.matmul(H20, A3)
    Z00 = np.array([0, 0, 1])
    Z10 = H10[0:3, 2]
    Z20 = H20[0:3, 2]
    O30 = H30[0:3, 3]
    O20 = H20[0:3, 3]
    O10 = H10[0:3, 3]

    # Compute Jacobian
    JV1 = np.cross(Z00, O30)
    JW1 = Z00
    JV2 = np.cross(Z10, (O30 - O10))
    JW2 = Z10
    JV3 = Z20
    JW3 = np.array([0, 0, 0])
    # print(np.shape(JV1))
    # print(np.shape(JW1))
    J1 = np.concatenate((JV1, JW1))
    J2 = np.concatenate((JV2, JW2))
    J3 = np.concatenate((JV3, JW3))
    J = [J1, J2, J3]
    J = np.transpose(J)
    # print("J_code:")
    # print(J)

    # J_1 = [-a2*sin(q1)*cos(q2)-a2*cos(q1)*cos(q2)-a1*sin(q1), -a2*sin(q1)*cos(q2)-a2*cos(q1)*sin(q2), 0]
    # J_2 = [a2*cos(q1)*cos(q2)-a2*sin(q1)*sin(q2)+a1*cos(q1), a2*cos(q1)*cos(q2)-a2*sin(q1)*sin(q2), 0]

    # J_ = [J_1,
    #     J_2,
    #     [0, 0, -1],
    #     [0, 0, 0],
    #     [0, 0, 0],
    #     [1, 1, 0]]
    # J_ = np.array(J_)
    # print("J: ")
    # print(J_)

def jacob_function(req):
    # Callback function for Service call to the service: Converts joint velocities to End Effector velocities
    q = req.joints.velocity
    
    print("J: ")
    print(J)
    
    print("Q: ")
    Q = np.array(q)
    print(Q)
    # print(np.shape(Q))

    # Matrix multiplication to compute twist from  Jacobian and Joint Angles
    print("E: ")
    E = np.matmul(J, Q)
    print(E)
    # print(np.shape(E))

    twist = Twist()
    twist.linear.x = E[0]
    twist.linear.y = E[1]
    twist.linear.z = E[2]
    twist.angular.x = E[3]
    twist.angular.y = E[4]
    twist.angular.z = E[5]
    return twist

def inv_jacob_function(req):
    # Callback function for Service call to the service: Converts End effector velocities to Joint velocities
    x_vel = req.twist.linear.x
    y_vel = req.twist.linear.y
    z_vel = req.twist.linear.z
    x_ang = req.twist.angular.x
    y_ang = req.twist.angular.y
    z_ang = req.twist.angular.z

    # Calculate Pseudo inverse of J
    print("J_inv: ")
    J_inv = np.linalg.pinv(J)
    print(J_inv)

    # Twist received from request

    E = np.array([x_vel, y_vel, z_vel, x_ang, y_ang, z_ang])
    print(E)
    # print(np.shape(E))

    # Find Joint angles
    Q = np.matmul(J_inv, E)
    print(Q)

    joints = JointState()
    joints.name = ["joint1", "joint2", "joint3"]
    joints.velocity = Q
    return joints

def vel_kinem():
    # Function to create subscriber and two services 
    rospy.init_node('vel_kinem')
    rospy.Subscriber("/scara_robot/joint_states", JointState, get_Jacob)
    s1 = rospy.Service('jacob', jacob, jacob_function)
    s2 = rospy.Service('inv_jacob', inv_jacob, inv_jacob_function)
    print("Started Velocity Kinematics server")
    rospy.spin()

if __name__ == "__main__":
    vel_kinem()
