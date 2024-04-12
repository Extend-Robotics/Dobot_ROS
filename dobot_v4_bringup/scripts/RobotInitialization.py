#!/usr/bin/env python

import rosnode
import rospy
import sys
import copy
import rospkg
from std_msgs.msg import String
import dobot_v4_bringup.srv
import time
from dobot_v4_bringup.srv import ClearErrorRequest, EnableRobotRequest

#Creating the ros node and service client
rospy.init_node("dobot_arm_enabler")
rospy.wait_for_service("/dobot_v4_bringup/srv/ClearError")
rospy.wait_for_service("/dobot_v4_bringup/srv/EnableRobot")

def EnableRobot():
    time.sleep(2)
    clear_error_service = rospy.ServiceProxy("/dobot_v4_bringup/srv/ClearError", dobot_v4_bringup.srv.ClearError)

    clear_error_response = clear_error_service()
    if(clear_error_response.res == 0):
        time.sleep(2)
        enable_robot_service = rospy.ServiceProxy("/dobot_v4_bringup/srv/EnableRobot", dobot_v4_bringup.srv.EnableRobot)
        enable_robot_response = enable_robot_service()
        if (enable_robot_response.res != 0):
            EnableRobot()
    else:
        EnableRobot()

if __name__ == '__main__':
    EnableRobot()
