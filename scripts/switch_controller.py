import math
import random
import rospy
from controller_manager_msgs.srv import SwitchController
from std_msgs.msg import Float64

def switcher():
    # initialize the node
    # at node initialization, the position controller is active
    # so only position(joint angles) commands can be given to joints
    rospy.init_node('switcher', anonymous=True)
    rate = rospy.Rate(1)  # meaning 1 message published in 1 sec
    rospy.sleep(5)
    random.seed()

    # once the joints have moved from home position,
    # the position controller is stopped and velocity controller is started.
    # We use ros inbuilt switch_controller service for that.
    rospy.wait_for_service('/scara_robot/controller_manager/switch_controller')
    try:
        sc_service = rospy.ServiceProxy(
            '/scara_robot/controller_manager/switch_controller', SwitchController)
        start_controllers = ['joint1_velocity_controller',
                             'joint2_velocity_controller']
        stop_controllers = ['joint1_position_controller',
                            'joint2_position_controller']
        strictness = 2
        start_asap = False
        timeout = 0.0
        res = sc_service(start_controllers, stop_controllers,
                         strictness, start_asap, timeout)

    except rospy.ServiceException as e:
        print("Service Call Failed")


if __name__ == '__main__':
    try:
        switcher()
        rospy.sleep(30)
    except rospy.ROSInterruptException:
        pass