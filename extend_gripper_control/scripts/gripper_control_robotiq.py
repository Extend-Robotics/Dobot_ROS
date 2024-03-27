#!/usr/bin/env python

import rosnode
import rospy
import sys
import copy
import rospkg
import extend_msgs
from extend_msgs.msg import GripperControl
from std_msgs.msg import String
import dobot_v4_bringup.srv

from dobot_v4_bringup.srv import ModbusCreateRequest, SetHoldRegsRequest

#Creating the ros node and service client
rospy.init_node("robotiq_gripper")
rospy.wait_for_service("/dobot_v4_bringup/srv/ModbusCreate")
rospy.wait_for_service("/dobot_v4_bringup/srv/SetHoldRegs")

def dataCallback(msg):
    # Remaping Range [0,1] to [0,255]
    gripper_value = 255 * msg.gripperAnalog.data
    gripper_modbus_service = rospy.ServiceProxy("/dobot_v4_bringup/srv/SetHoldRegs", dobot_v4_bringup.srv.SetHoldRegs)
    gripper_modbus_data = SetHoldRegsRequest()
    gripper_modbus_data.index = 0
    gripper_modbus_data.addr = 1000
    gripper_modbus_data.count = 3
    gripper_modbus_data.valTab = "{2304," + gripper_value + ",60000}"
    gripper_modbus_data.valType = "U16"
    gripper_modbus_service(gripper_modbus_data)



if __name__ == '__main__':
    #Configure the modbus
    modbus_create_service = rospy.ServiceProxy("/dobot_v4_bringup/srv/ModbusCreate", dobot_v4_bringup.srv.ModbusCreate)
    modbus_config = ModbusCreateRequest()
    modbus_config.ip = "127.0.0.1"  #IP of the gripper
    modbus_config.port = 60000  #Gripper Connection Port
    modbus_config.slave_id = 9  #Glipper Port Slave ID
    modbus_config.isRTU = 1  #RTU
    modbus_create_service(modbus_config)

    #Activate Gripper
    gripper_modbus_service = rospy.ServiceProxy("/dobot_v4_bringup/srv/SetHoldRegs", dobot_v4_bringup.srv.SetHoldRegs)
    gripper_modbus_data = SetHoldRegsRequest()
    gripper_modbus_data.index = 0
    gripper_modbus_data.addr = 1000
    gripper_modbus_data.count = 3
    gripper_modbus_data.valTab = "{0,0,0}"
    gripper_modbus_data.valType = "U16"
    gripper_modbus_service(gripper_modbus_data)


    gripper_modbus_data = SetHoldRegsRequest()
    gripper_modbus_data.index = 0
    gripper_modbus_data.addr = 1000
    gripper_modbus_data.count = 3
    gripper_modbus_data.valTab = "{2034,0,0}"
    gripper_modbus_data.valType = "U16"
    gripper_modbus_service(gripper_modbus_data)

    #Subscribe to Digital Gripper Data Stream from Unity
    rospy.Subscriber("/extend_gripper_command", GripperControl, dataCallback, queue_size=1)
    rospy.spin()
