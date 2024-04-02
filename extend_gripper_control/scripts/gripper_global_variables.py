#!/usr/bin/env python

import rospy

def initialize_global():
    print("Global Variables initialized")

    global gripper_prev_state,gripper_initialization

    gripper_prev_state = 0
    gripper_initialization = False
