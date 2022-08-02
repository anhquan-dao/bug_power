#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import rosparam
from sensor_msgs.msg import BatteryState
from bug_power.driver import Driver

import os

import time

if __name__ == "__main__":
    rospy.init_node("test")

    battery_pub = rospy.Publisher("battery", BatteryState, queue_size=1)
    battery_state = BatteryState()

    param_list = rosparam.list_params(rospy.get_name())
    print(rospy.get_name())
    params = {}
    for i in range(len(param_list)):
        param_list[i] = param_list[i].replace(rospy.get_name()+"/", "", 1)
        params[param_list[i]] = rospy.get_param("~"+param_list[i])

    print(params)
    driver = Driver(params)

    while not rospy.is_shutdown():
        driver.readMessage()
        battery_state.voltage = driver.voltage
        # if(driver.voltage < 11.5):
        #     node_to_kill = ""
        #     for i in params["node_to_kill"]:
        #         node_to_kill += " " + i
        #     os.system("rosnode kill" + node_to_kill + " > /dev/null 2>&1")

        battery_pub.publish(battery_state)
        rospy.sleep(0.01)
        
