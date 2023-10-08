#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from rebet_msgs.msg import AdaptationState
from std_msgs.msg import Float64

from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
import numpy as np

from rebet.adaptation_logic import AdaptationLogic

class RandomActionServer(AdaptationLogic):

    def __init__(self):
        super().__init__('random_action_server')
        
        self.cli = self.create_client(SetParameters, '/identify_action_server/set_parameters', callback_group=MutuallyExclusiveCallbackGroup())
        self.req = SetParameters.Request()
        self.time_to_adapt = False
        self.current_pic_rate = 0
        self.get_logger().info('Random-based Action server created...')

    

    def send_set_parameters(self):
        while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('set_param service not available, waiting again...')
        response = self.cli.call(self.req)

        if(not all([res.successful for res in response.results])):
            self.get_logger().warning('One or more requests to set a parameter were unsuccessful in Random Adaptation Logic, see reason(s):' + str(response.results))

        self.get_logger().info('Param set...')



    def execute_logic(self):
        self.get_logger().info('Executing goal...')
        

        self.get_logger().info('Random-based action server is waiting for the moment to adapt',throttle_duration_sec=5)
        
        random_config = np.random.choice(self.possible_configs)

        self.req.parameters = random_config.configuration_parameters

        self.current_pic_rate = random_config.configuration_parameters[0].value.integer_value

        self.send_set_parameters()
        
        self.get_logger().info('Random-based action server adapted the system to parameter value ' + str(self.current_pic_rate) ,throttle_duration_sec=5)

    
    def adaptation_state_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)
        if(msg is not None):
            self.possible_configs = msg.system_possible_configurations
            self.time_to_adapt = True

            self.execute_logic()

        




def main(args=None):
    rclpy.init(args=args)

    random_action_server = RandomActionServer()
    
    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(random_action_server)
    mt_executor.spin()

    random_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()