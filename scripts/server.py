#!/usr/bin/env python3 	

from grouphw.srv import inv, invResponse
import rospy
import numpy as np
import math
from sensor_msgs.msg import JointState

# a0 + a1 are the lengths of the two segments of link1, 
# a2 is the length of link2
# h0 is the base cylinder offset
a1 = 1
a2 = 1
a0 = 0.5
h0 = 0.3

def handle_inverse_kin(req):
    x = req.pose.position.x
    y = req.pose.position.y
    z = req.pose.position.z

    D = ((x**2)+(y**2)-(a1**2)-(a2**2))/(2*a1*a2)
    print(D, x, y, z)
    q2 = math.atan2(math.sqrt(1-D**2), D)
    q1 = np.arctan2(y, x) - np.arctan2(a2*math.sin(q2), (a1 + a2*math.cos(q2)))
    d3 = (h0 + a0 - z)

    joints = JointState()
    joints.name = ["joint1", "joint2", "joint3"]
    joints.position = [q1, q2, d3]
    return joints

def inv_kin_server():
    rospy.init_node('server')
    s = rospy.Service('inv', inv, handle_inverse_kin)
    print("Inverse Kinematics: ")
    rospy.spin()

if __name__ == "__main__":
    inv_kin_server()
