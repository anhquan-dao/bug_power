#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import rosparam
from sensor_msgs.msg import BatteryState
from bug_power.driver import TestDriver

import os

import time

if __name__ == "__main__":
    rospy.init_node("test")

    driver = TestDriver()

    while not rospy.is_shutdown():
        error = driver.readMessage(data_cb=driver.data_callback)
        if(error < 0):
            if(driver.disconnected == True):
                rospy.logerr_throttle(1, "Lost connection to MCU")
            else:
                rospy.logwarn("Error when reading message. Error code: " + str(error))
            
            continue
        
        driver.battery_state.voltage = driver.bus_voltage
        driver.battery_state.current = driver.current
        driver.battery_state.capacity = driver.capacity
        
        driver.battery_pub.publish(driver.battery_state)

        
