#!/usr/bin/env python3

from rebet_msgs.srv import SetAttributesInBlackboard
from rebet_msgs.msg import SystemAttributeValue, SystemAttribute
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


class SystemReflection(Node):
    
    def __init__(self):
        print("Initializing the node...")
        
        super().__init__('system_reflection')
        
        timer_cb_group = MutuallyExclusiveCallbackGroup()
        exclusive_group = MutuallyExclusiveCallbackGroup()

        self.cli = self.create_client(SetAttributesInBlackboard, '/set_attributes_in_blackboard', callback_group=exclusive_group)

        self.laserscan_subscription = self.create_subscription(LaserScan,'/scan_raw',self.ls_scan_cb,1,callback_group=MutuallyExclusiveCallbackGroup())
        self.odometry_subscription = self.create_subscription(Odometry,'/ground_truth_odom',self.tb_odom_cb,10, callback_group = MutuallyExclusiveCallbackGroup())
        self.lighting_subscription = self.create_subscription(Float32,'/current_lighting',self.current_lighting_cb,10, callback_group = MutuallyExclusiveCallbackGroup())

        self.req = SetAttributesInBlackboard.Request()
        self.odom_msg = None
        self.laser_msg = None
        self.current_lighting_msg = None

        self.time_monitor_timer = self.create_timer(2, self.process_and_send, callback_group=timer_cb_group)        

    def send_request(self, script):
        print("sending req")
        with self.battery_state_lock:
            while not self.cli.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('service not available, waiting again...')
            self.req.script_code = script 
            self.future = self.cli.call_async(self.req)
        
    def ls_scan_cb(self,msg):
        self.laser_msg = msg

    def current_lighting_cb(self, msg):
        self.current_lighting_msg = msg

    def tb_odom_cb(self,msg):
        self.odom_msg = msg


    def process_and_send(self):
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

        if(self.current_lighting_msg is not None):
            self.get_logger().info("Lighting msg")
            new_sys_att = SystemAttribute()
            new_sys_att.name = 'current_lighting'

            att_value = SystemAttributeValue()
            att_value.header.stamp = self.get_clock().now().to_msg()
            att_value.type = 4 #float type
            att_value.float_value = self.current_lighting_msg
            new_sys_att.value = att_value 
            self.req.sys_attributes.append(new_sys_att)

            self.get_logger().info(str(new_sys_att))

            self.current_lighting_msg = None
        

        if(len(self.req.sys_attributes) > 0):
            
            self.get_logger().info("Trying to call set att...")

            while not self.cli.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info('service not available, waiting again...')
            response = self.cli.call(self.req)
            self.get_logger().info('Result of call to set attribute in blackboard ' + str(response.success))
            self.req.sys_attributes = []

        self.get_logger().info("Finish sys reflec calls...")
        
   
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