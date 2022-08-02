#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import rosparam
from sensor_msgs.msg import BatteryState
from base_mcu_interface.base_driver import BaseDriver

class TestDriver(BaseDriver):
        def __init__(self, params=None):
            
            if(params == None):
                param_list = rosparam.list_params(rospy.get_name())
                params = dict()
                for i in range(len(param_list)):
                    param_list[i] = param_list[i].replace(rospy.get_name()+"/", "", 1)
                    params[param_list[i]] = rospy.get_param("~"+param_list[i])

            params["header"] = [0x97, 0x96, 0x95, 0x94]
            
            BaseDriver.__init__(self, params)

            self.max_voltage = params["max_voltage"]
            
            self.float_data = 0.0
            self.bus_voltage = 0.0
            self.current = 0.0
            self.shunt_votlage = 0.0
            self.capacity = 0.0

            self.battery_pub = rospy.Publisher("battery", BatteryState, queue_size=1)
            self.battery_state = BatteryState()            

        def data_callback(self, header, data):                
            self.float_data = self.parseFloat()
            if(header == 0x97):
                self.bus_voltage = self.float_data
            if(header == 0x96):
                self.current = self.float_data
            if(header == 0x95):
                self.shunt_voltage = self.float_data
            if(header == 0x94):
                self.capacity = self.float_data
            
        def error_callback(self, error):
            print("Test pass method as argument")
            pass