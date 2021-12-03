#!/usr/bin/env python3

from time import sleep, time
import rospy
from matplotlib import pyplot
from rospy.rostime import get_time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock
import numpy as np

# joint = 1
step = 0
joint_vals = []
time_arr = []
start_time = 0.0
time_now = 0.0

def plot():
    global joint_vals, time_arr
    ref_vals = np.ones((len(joint_vals), 1), np.float) - 0.3
    pyplot.plot(time_arr, joint_vals, 'g-', label="Current Joint Value")
    pyplot.plot(time_arr, ref_vals, 'r-', label="Reference Joint Value")
    pyplot.xlabel('Time (seconds)')
    pyplot.ylabel('Joint Value (radians)')
    pyplot.grid(True)
    pyplot.legend(loc="upper right")
    pyplot.show()

def get_joint_state(data):
    # This callback function gets the joint states and stores them in an array
    # if the step flag is high and time data is being logged
    global start_time, step, time_now, joint_vals, time_arr
    curr_val = data.position[0]
    # print("Current value: ", curr_val)
    if step:
        if start_time == 0:
            start_time = time_now
        else:
            time_diff = time_now - start_time
            if time_diff <= 10.0:
                joint_vals.append(curr_val)
                time_arr.append(time_diff)
            else:
                plot()
                step = 0
        

def get_time(time_clk):
    # This callback function gets the time in seconds and nanoseconds
    # It stores these values in a global variable
    global time_now
    sec = time_clk.clock.secs
    nsec = time_clk.clock.nsecs / 1000000000
    time_now = sec + nsec

def pub_set_pt():
    # This function publishes the set point to the joint and sets the step flag to true
    global step
    pub = rospy.Publisher("/scara_robot/joint1_position_controller/command", Float64, queue_size=1)
    sleep(1)
    val = 0.7
    pub.publish(val)
    rospy.loginfo("Publish value: %f", val)
    step = 1
    rospy.loginfo("Moving the robot joint and generating plot")
    rospy.spin()

if __name__=="__main__":
    rospy.init_node('tuner', anonymous=True)
    rospy.Subscriber("/scara_robot/joint_states", JointState, get_joint_state)
    rospy.Subscriber("/clock", Clock, get_time)
    rate = rospy.Rate(10)
    pub_set_pt()