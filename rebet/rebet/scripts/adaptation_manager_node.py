#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rebet_msgs.srv import GetQR, GetVariableParams, SetBlackboard, AdaptSystem, RequestAdaptation, SetParameterInBlackboard
from rcl_interfaces.msg import Parameter
from std_msgs.msg import Float64
from rebet_msgs.msg import AdaptationState, Configuration, QRValue
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rebet.adaptation_strategies import create_strategy
from itertools import product
import numpy as np
import sys

ADAP_PERIOD_PARAM = "adaptation_period"



class AdaptationManager(Node):

    def __init__(self): 
        super().__init__('adaptation_manager')
        self.publisher_ = self.create_publisher(AdaptationState, 'system_adaptation_state', 10)

        self.task_to_strategy_map = {}
        self.i = 0
        exclusive_group = MutuallyExclusiveCallbackGroup()
        self.declare_parameter(ADAP_PERIOD_PARAM, 8)
        self.adaptation_period = self.get_parameter(ADAP_PERIOD_PARAM).get_parameter_value().integer_value

        # self.timer = self.create_timer(self.adaptation_period, self.timer_callback)
        self.srv_adapt = self.create_service(RequestAdaptation, '/request_adaptation',self.adaptation_requested)
        
        self.cli_exec = self.create_client(SetParameterInBlackboard, '/set_parameter_in_blackboard', callback_group=exclusive_group)
        self.req_exec = SetParameterInBlackboard.Request()


        self.cli_qr = self.create_client(GetQR, '/get_qr', callback_group=exclusive_group)
        
        self.cli_sbb = self.create_client(SetBlackboard, '/set_blackboard', callback_group=exclusive_group)

        self.req_sbb = SetBlackboard.Request()
        self.req_sbb.script_code = "average_utility:='"
        self.req_qr = GetQR.Request()
        self.req_var = GetVariableParams.Request()

        self.reporting = [0,0]

        self.bounds_dict = {}


    def dynamic_bounding(self, utility, bounds):
        lower_bound, upper_bound = bounds


        if(utility > upper_bound): upper_bound = utility


        elif(utility < lower_bound): lower_bound = utility

        new_range = upper_bound - lower_bound

        
        # self.get_logger().info("new_range " + str(new_range))
        # self.get_logger().info("new bounds " + str((lower_bound,upper_bound)))
        # self.get_logger().info("bounds " + str(bounds))
        # self.get_logger().info("utility " + str(utility))

        

        result = float((utility - lower_bound)/new_range)

        bounds[0:2] = [lower_bound,upper_bound]

        return result


    def get_system_utility(self):
        self.get_logger().info("Calling QR service client...")
        
        while not self.cli_qr.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        response = self.cli_qr.call(self.req_qr)

        self.get_logger().info('Result of QRS in tree ' + str(response.qrs_in_tree))


        weight_sum = sum([qr.weight for qr in response.qrs_in_tree])


        all_the_qrs = []

        for qr in response.qrs_in_tree:
            if qr.qr_name not in self.bounds_dict: self.bounds_dict[qr.qr_name] = [0,0.000000000001]

            normalized_value = self.dynamic_bounding(qr.metric, self.bounds_dict[qr.qr_name])
            self.get_logger().info("Bounds dict: " + str(self.bounds_dict))
            self.get_logger().info("Metric value vs. normalized value " + str(qr.metric) + " vs. " + str(normalized_value))

            qr_val = QRValue()
            qr_val.name = qr.qr_name
            qr_val.qr_fulfilment = (qr.weight/weight_sum) * normalized_value
            all_the_qrs.append(qr_val)

            
        self.reporting[0]+=(np.product([qr.qr_fulfilment for qr in all_the_qrs]))
        self.reporting[1]+=1
        self.req_sbb.script_code+=str(self.reporting[0]/self.reporting[1]) + "'"

        self.get_logger().info(self.req_sbb.script_code)

        res = self.cli_sbb.call(self.req_sbb)
        self.get_logger().info("Put this in the whiteboard for average utility " + str(self.reporting[0]/self.reporting[1]) + " with res " + str(res.success))

        self.req_sbb.script_code = "average_utility:='"
        return all_the_qrs
    
    def make_configurations(self, variable_parameters_obj):
        param_to_node = {}

        
        possible_configurations = []

        possible_configurations = []
        list_of_list_param = []
        for knob in variable_parameters_obj.variable_parameters:
            decomposed = []
            #knob is a msg of type VariableParameter is name-potential_values pair representing a thing that can change about the current state of the system.
            for pos_val in knob.possible_values:
                param = Parameter()
                param.name = knob.name
                param.value = pos_val
                param_to_node[str((param.name, param.value))] = knob.node_name
                decomposed.append(param)
            list_of_list_param.append(decomposed)
        
        self.get_logger().info('list of list param ' + str(list_of_list_param))
        possible_configurations = list(product(*list_of_list_param))
        #here's where you'd apply constraints to remove invalid configurations
        print(len(possible_configurations))
        print(possible_configurations)
        config_list = []
        for possible_config in possible_configurations:
            if(len(possible_config) != 0):
                possible_config_list = list(possible_config)
                config_msg = Configuration()
                config_msg.node_names = [param_to_node[str((param.name, param.value))] for param in possible_config_list]
                config_msg.configuration_parameters = possible_config
                config_list.append(config_msg)
        #The arms should consists of a list of Parameter, name value pairs of each parameter given.
        return config_list


    def execute_adaptation(self):

        while not self.cli_exec.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set param in bb service not available, waiting again...')
        
        response = self.cli_exec.call(self.req_exec)    

        return response.success

    def timer_callback(self):        
        #I should probably make functions like these into utilities, like as members of a subclass of Node..




        
        msg = AdaptationState()
        msg.qr_values = self.get_system_utility()
        msg.system_possible_configurations = self.get_system_vars()
        self.publisher_.publish(msg)
        self.get_logger().info('\n\n\nPublishing: "%s"\n\n\n\n\n' % msg)
        self.i += 1

    def adaptation_requested(self, request, response):
        task_identifier = str(request.task_identifier)
        adaptation_strategy = str(request.adaptation_strategy)
        self.get_logger().info('\n\n\hi:\n\n\n\n\n')

        response.success = False
        adapt_state = AdaptationState()
        self.get_logger().info('\n\n pre sys util \n\n')

        adapt_state.qr_values = self.get_system_utility()
        self.get_logger().info('\n\n sys util \n\n')

        adapt_state.system_possible_configurations = self.make_configurations(request.adaptation_options)

        self.get_logger().info(str(adapt_state.system_possible_configurations))
        
        
        # all_the_qrs = []

        # for i in range(5):
        #     qr_val = QRValue()
        #     qr_val.name = "test"
        #     qr_val.qr_fulfilment = 5.0
        #     all_the_qrs.append(qr_val)

        # msg.qr_values = all_the_qrs

        #new task or new strategy for the same task.

        condition_one = False
        condition_two = False


        if ( (task_identifier not in self.task_to_strategy_map) or ( (task_identifier in self.task_to_strategy_map) and (adaptation_strategy != self.task_to_strategy_map[task_identifier].get_name()) ) ):
            self.task_to_strategy_map[task_identifier] = create_strategy(adaptation_strategy)
        elif(request.adaptation_strategy == "reset"):
            #reset of same strategy during task.
            self.get_logger().info('\n\n RESET OF STRATEGY \n\n')

            self.task_to_strategy_map[task_identifier] = create_strategy(self.task_to_strategy_map[task_identifier].get_name())
        else:
            self.get_logger().info('\n\n REUSE OF STRATEGY \n\n')



        
        self.req_exec.ros_parameters = self.task_to_strategy_map[task_identifier].suggest_adaptation(adapt_state).configuration_parameters
        is_exec_success = self.execute_adaptation()

        response.success = is_exec_success
        
        return response


    


    

def main(args=None):
    rclpy.init()

    adapt_manage_node = AdaptationManager()

    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(adapt_manage_node)
    
    mt_executor.spin()
   

    
    
    adapt_manage_node.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
    main()