#!/usr/bin/env python3

from rclpy.executors import ExternalShutdownException
from gazebo_msgs.srv import SetLightProperties
import rclpy
from rclpy.node import Node
import math
import time
from std_msgs.msg import Float32, ColorRGBA
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class ChangeLightingClient(Node):
    def __init__(self):

        super().__init__('change_lighting_client')
        self.cli = self.create_client(SetLightProperties, '/small_warehouse/set_light_properties',callback_group=MutuallyExclusiveCallbackGroup())
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetLightProperties.Request()

        self.pub_light = self.create_publisher(Float32,'/current_lighting',10,callback_group=MutuallyExclusiveCallbackGroup())

        timer_period = 1  # seconds

        self.timer = self.create_timer(timer_period, self.timer_cb, callback_group=MutuallyExclusiveCallbackGroup())
        self.start_time = time.time()
        self.current_light = 127.0
        self.count = 0

    def timer_cb(self):
        self.count = self.count + 1
        float_msg = Float32()
        float_msg.data = self.current_light / 127

        self.pub_light.publish(float_msg)
        if(self.count > 5):
            self.count = 0
            self.send_request()

    def send_request(self):
        if(self.current_light == 1.1):
            self.current_light = 127.0
        elif(self.current_light == 127.0):
            self.current_light = 1.1
        
        self.get_logger().info('Changing diffusion to: %d' % (self.current_light))
        self.req.light_name = "Warehouse_CeilingLight_003"
        new_color = ColorRGBA()

        new_color.r = self.current_light
        new_color.g = self.current_light
        new_color.b = self.current_light
        new_color.a = 255.0

        self.req.diffuse = new_color

        self.req.attenuation_constant = 0.30
        self.req.attenuation_linear = 0.01
        self.req.attenuation_quadratic = 0.0

        response = self.cli.call(self.req)
        self.get_logger().info('Result of call to set light properties' + str(response.success))
        self.get_logger().info('Result of call to set light properties' + str(response.status_message))



    
def main(args=None):
    rclpy.init(args=args)

    node = ChangeLightingClient()
    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(node)
    
    mt_executor.spin()

if __name__ == '__main__':
    main()