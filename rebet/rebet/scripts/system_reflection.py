#!/usr/bin/env python3
import sys

from rebet_msgs.srv import SetAttributesInBlackboard
from rebet_msgs.msg import SystemAttributeType, SystemAttributeValue, SystemAttribute

from sensor_msgs.msg import BatteryState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import rclpy
from rclpy.node import Node
import time
from rclpy.executors import MultiThreadedExecutor
from collections import deque
import queue
#Essentially this is client to the setBlackboard service with some logic around it to incorporate different
#published data about the system.
import time
import threading
from nav_msgs.msg import Odometry
from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import KeyValue
from std_msgs.msg import Float32

import numpy as np

class SystemReflection(Node):
    
    def __init__(self):
        print("Initializing the node...")
        #self.node_name = 'system_reflection'
        super().__init__('system_reflection')
        
        timer_cb_group = MutuallyExclusiveCallbackGroup()
        exclusive_group = MutuallyExclusiveCallbackGroup()
        sub_group = MutuallyExclusiveCallbackGroup()
        self.counter = 0
        self.diag_counter = 0

        self.cli = self.create_client(SetAttributesInBlackboard, '/set_attributes_in_blackboard', callback_group=exclusive_group)
        self.subscription = self.create_subscription(BatteryState,'/battery/status',self.battery_state_cb,10, callback_group=sub_group)

        self.diagnostic_subscription = self.create_subscription(DiagnosticArray,'/diagnostics',self.sv_diag_cb,1,callback_group=MutuallyExclusiveCallbackGroup())
        self.laserscan_subscription = self.create_subscription(LaserScan,'/scan',self.ls_scan_cb,1,callback_group=MutuallyExclusiveCallbackGroup())
        self.odometry_subscription = self.create_subscription(Odometry,'/odom',self.tb_odom_cb,10, callback_group = MutuallyExclusiveCallbackGroup())
        
        self.pipeline_distance_inspected_sub = self.create_subscription(Float32,'pipeline/distance_inspected',self.distance_inspected_cb,1,callback_group=MutuallyExclusiveCallbackGroup())

        self.distance_inspected_msg = None
        self.req = SetAttributesInBlackboard.Request()
        self.odom_msg = None
        self.laser_msg = None
        self.diagnostic_values = queue.Queue() #queue of diagnostic KeyValue
        self.time_monitor_timer = self.create_timer(2, self.do_stuff, callback_group=timer_cb_group)
        self.battery_state = None
        self.state_queue = deque(maxlen=20) #just picked a random length, can be adjusted..
        self.diag_queue_lock = threading.Lock()

        

    def send_request(self, script):
        print("sending req")
        with self.battery_state_lock:
            while not self.cli.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('service not available, waiting again...')
            self.req.script_code = script 
            self.future = self.cli.call_async(self.req)
            # while rclpy.spin_until_future_complete(self, self.future):
            #     self.get_logger().info("Waiting for future to complete")
            # return self.future.result()
        
    def ls_scan_cb(self,msg):
        self.laser_msg = msg
        self.counter+=1
        if(self.counter % 1000 == 0): self.get_logger().info('1000th Laser Msg Received!!')
        if self.counter > 200000: self.counter = 0

    def distance_inspected_cb(self, msg):
        self.distance_inspected_msg = msg

    def tb_odom_cb(self,msg):
        self.odom_msg = msg
        #self.linear_speed = np.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.counter+=1
        if(self.counter % 1000 == 0): self.get_logger().info('1000th Odom Msg Received!!')
        if self.counter > 200000: self.counter = 0

    def sv_diag_cb(self,msg):
        #self.linear_speed = np.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        self.diag_counter+=1

        if(self.counter % 10000 == 0): self.get_logger().info('10000th Diagnostics Msg Received!!')
        if self.counter > 200000: self.counter = 0

        for diag_status in msg.status:
            if(diag_status.message == "QA status" or diag_status.message == "Component status"):
                for diag_value in diag_status.values:
                    with self.diag_queue_lock:
                        self.diagnostic_values.put_nowait(diag_value)
            



    def battery_state_cb(self,msg):
        self.state_queue.append(msg)

    def do_stuff(self):
        #print('lpp'+ str(i))
        self.get_logger().info("Processing battery state and calling service client...")
        
        #script = self.process_bt_state()
        #print("\nscript is " + script + "\n\n")
        #if(script != ""):
            # self.req.script_code = script 
            # response = self.cli.call(self.req)

            # self.get_logger().info('Result of it ' + str(response.success))

            
            # self.battery_state = None

        if(self.odom_msg is not None):
            new_sys_att = SystemAttribute()
            new_sys_att.name = 'odometry'

            att_value = SystemAttributeValue()
            att_value.header.stamp = self.get_clock().now().to_msg()
            att_value.type = 1 #odometry type
            att_value.odom_value = self.odom_msg
            new_sys_att.value = att_value 
            self.req.sys_attributes.append(new_sys_att)

            self.odom_msg = None



        if(self.laser_msg is not None):
            new_sys_att = SystemAttribute()
            new_sys_att.name = 'laser_scan'

            att_value = SystemAttributeValue()
            att_value.header.stamp = self.get_clock().now().to_msg()
            att_value.type = 3 #laserscan type
            att_value.laser_value = self.laser_msg
            new_sys_att.value = att_value 
            self.req.sys_attributes.append(new_sys_att)

            self.laser_msg = None

        if(self.distance_inspected_msg is not None):
            new_sys_att = SystemAttribute()
            new_sys_att.name = 'distance_inspected'

            att_value = SystemAttributeValue()
            att_value.header.stamp = self.get_clock().now().to_msg()
            att_value.type = 4 #float type
            att_value.float_value = self.distance_inspected_msg
            new_sys_att.value = att_value 
            self.req.sys_attributes.append(new_sys_att)

            self.distance_inspected_msg = None


            # response = self.cli.call(self.req)

        
        with self.diag_queue_lock:
            while not self.diagnostic_values.empty():
                diag_keyvalue = self.diagnostic_values.get_nowait()
                new_sys_att = SystemAttribute()
                
                new_sys_att.name = diag_keyvalue.key

                att_value = SystemAttributeValue()
                att_value.header.stamp = self.get_clock().now().to_msg()
                att_value.type = 2 #diagnositc keyvalue type
                att_value.diag_value = diag_keyvalue
                new_sys_att.value = att_value 
                self.req.sys_attributes.append(new_sys_att)

        if(len(self.req.sys_attributes) > 0):
            self.get_logger().info("Trying to call set att...")

            while not self.cli.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('service not available, waiting again...')
            response = self.cli.call(self.req)
            self.get_logger().info('Result of call to set attribute in blackboard ' + str(response.success))
            self.req.sys_attributes = []

        self.get_logger().info("Finish sys reflec calls...")
        


        

    
    def script_string(self, field, value):
        assignment_string = ""
        assignment_string+= field
        assignment_string+= ":="
        if type(value) == str:
            assignment_string+= "'" + value + "'"
        else:
            assignment_string+= str(value)
            assignment_string+= "; "
        return assignment_string
        

    def process_bt_state(self):
        #float32 voltage          # Voltage in Volts (Mandatory)
        #float32 temperature      # Temperature in Degrees Celsius (If unmeasured NaN)
        #float32 current          # Negative when discharging (A)  (If unmeasured NaN)
        #float32 charge           # Current charge in Ah  (If unmeasured NaN)
        #float32 capacity         # Capacity in Ah (last full capacity)  (If unmeasured NaN)
        #float32 design_capacity  # Capacity in Ah (design capacity)  (If unmeasured NaN)
        #float32 percentage       # Charge percentage on 0 to 1 range  (If unmeasured NaN)
        
        #float32 linear_speed 
        try:
            bt_state = self.state_queue[-1]
            assignment_string = ""
            fields = ["voltage", "temperature", "current", "charge", "capacity", "design_capacity", "percentage"] #paramterize?
            for field in fields:
                assignment_string += self.script_string(field, getattr(bt_state,field))

            #assignment_string+= self.script_string("linear_speed", self.linear_speed)
            assignment_string = assignment_string[:-2]
            return assignment_string
        except IndexError:
            #nothing in the queue (yet?)
            return ""

def main():
    rclpy.init()

    sys_reflec_node = SystemReflection()

    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(sys_reflec_node)
    
    mt_executor.spin()
   

    
    
    sys_reflec_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()