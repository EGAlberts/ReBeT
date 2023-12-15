#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rebet_msgs.srv import GetQR, GetVariableParams, SetBlackboard, OnlineAdaptation, OfflineAdaptation, SetParameterInBlackboard
from rcl_interfaces.msg import Parameter
from std_msgs.msg import Float64
from rebet_msgs.msg import AdaptationState, Configuration, QRValue, Adaptation
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.srv import SetParameters
from rebet.adaptation_strategies import create_strategy
from lifecycle_msgs.srv import ChangeState, GetState
from itertools import product
import numpy as np
import sys

ADAP_PERIOD_PARAM = "adaptation_period"

ROS_LIFECYCLE = "ros_lifecycle"
ROS_PARAM = "ros_parameter"
BLACKBOARD = "blackboard"
ADAP_DICT = {
    0: ROS_LIFECYCLE,
    1: ROS_PARAM,
    2: BLACKBOARD
}


def value_from_param(param_msg):
    param_type = param_msg.value.type


    # uint8 PARAMETER_BOOL=1
    # uint8 PARAMETER_INTEGER=2
    # uint8 PARAMETER_DOUBLE=3
    # uint8 PARAMETER_STRING=4
    # uint8 PARAMETER_BYTE_ARRAY=5
    # uint8 PARAMETER_BOOL_ARRAY=6
    # uint8 PARAMETER_INTEGER_ARRAY=7
    # uint8 PARAMETER_DOUBLE_ARRAY=8
    # uint8 PARAMETER_STRING_ARRAY=9

    if(param_type == 1): return param_msg.value.bool_value
    if(param_type == 2): return param_msg.value.integer_value
    if(param_type == 3): return param_msg.value.double_value
    if(param_type == 4): return param_msg.value.string_value
    if(param_type == 5): return param_msg.value.byte_array_value
    if(param_type == 6): return param_msg.value.bool_array_value
    if(param_type == 7): return param_msg.value.integer_array_value
    if(param_type == 8): return param_msg.value.double_array_value
    if(param_type == 9): return param_msg.value.string_array_value

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
        self.srv_online_adapt = self.create_service(OnlineAdaptation, '/online_adaptation',self.online_adaptation_requested)
        self.srv_offline_adapt = self.create_service(OfflineAdaptation, '/offline_adaptation',self.offline_adaptation_requested)

        
        self.cli_bb_exec = self.create_client(SetParameterInBlackboard, '/set_parameter_in_blackboard', callback_group=exclusive_group)



        self.cli_qr = self.create_client(GetQR, '/get_qr', callback_group=exclusive_group)
        
        self.cli_sbb = self.create_client(SetBlackboard, '/set_blackboard', callback_group=exclusive_group)

        self.req_sbb = SetBlackboard.Request()
        self.req_sbb.script_code = "average_utility:='"
        self.req_var = GetVariableParams.Request()

        self.reporting = [0,0]

        self.bounds_dict = {}
        self.set_parameter_client_dict = {}
        self.change_state_client_dict = {}
        self.get_state_client_dict = {}
        self.reporting_dict = {}
        




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


    def get_system_utility(self, system_level=False):
        self.get_logger().info("Calling QR service client...")
        
        while not self.cli_qr.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        
        req_qr = GetQR.Request()
        
        req_qr.at_system_level = system_level


        response = self.cli_qr.call(req_qr)

        self.get_logger().info('Result of QRS in tree ' + str(response.qrs_in_tree))


        weight_sum = sum([qr.weight for qr in response.qrs_in_tree])


        all_the_qrs = []

        for qr in response.qrs_in_tree:
            if qr.qr_name not in self.bounds_dict: self.bounds_dict[qr.qr_name] = [0,0.000000000001]

            normalized_value = self.dynamic_bounding(qr.metric, self.bounds_dict[qr.qr_name])
            self.get_logger().info("Bounds dict: " + str(self.bounds_dict))
            self.get_logger().info("Metric value vs. normalized value " + str(qr.metric) + " vs. " + str(normalized_value))
            

            if(qr.higher_is_better == False):
               normalized_value = 1 - normalized_value
            qr_val = QRValue()
            qr_val.name = qr.qr_name
            qr_val.qr_fulfilment = (qr.weight/weight_sum) * normalized_value
            all_the_qrs.append(qr_val)

        
        return all_the_qrs
    

    def report_on_system(self, all_the_qrs):
        self.req_sbb.script_code = ""
        for key in list(self.reporting_dict.keys()):
            if((str(key) != "pic_rate") and (str(key) != "charge_or_not")):
                if(str(key) == "max_velocity"):
                    rep_val = str(self.reporting_dict[key][0])
                else:
                    rep_val = str(self.reporting_dict[key])
                self.req_sbb.script_code+=str(key)
                self.req_sbb.script_code+=":='"
                self.req_sbb.script_code+=rep_val
                self.req_sbb.script_code+="';"

            
        
        self.req_sbb.script_code+= "average_utility:='" 
        self.reporting[0]+=(np.product([qr.qr_fulfilment for qr in all_the_qrs]))
        self.reporting[1]+=1
        self.req_sbb.script_code+=str(self.reporting[0]/self.reporting[1]) + "'"




        self.get_logger().info(self.req_sbb.script_code)

        res = self.cli_sbb.call(self.req_sbb)
        self.get_logger().info("Put this in the whiteboard for average utility " + str(self.reporting[0]/self.reporting[1]) + " with res " + str(res.success))
    def make_configurations(self, adaptation_options_list):
        param_to_node = {}
        param_to_target = {}

        
        possible_configurations = []

        possible_configurations = []
        list_of_list_param = []
        for adaptation_options in adaptation_options_list:
            #AdaptationOptions.msg:
            # #name of the parameter
            # string name
            # #name of the node (if any) it belongs to
            # string node_name
            # #The type of adaptation being done to the target, this is constrained choice specified in Adaptation.msg
            # int8 adaptation_target_type
            # #A presumed finite set of acceptable values for the parameter to hold.
            # rcl_interfaces/ParameterValue[] possible_values 

            decomposed = []

            for pos_val in adaptation_options.possible_values:
                param = Parameter()
                param.name = adaptation_options.name
                param.value = pos_val
                param_to_node[str((param.name, param.value))] = adaptation_options.node_name
                param_to_target[str((param.name, param.value))] = adaptation_options.adaptation_target_type
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
                config_msg.adaptation_target_types = [param_to_target[str((param.name, param.value))] for param in possible_config_list]
                config_msg.configuration_parameters = possible_config
                config_list.append(config_msg)
        #The arms should consists of a list of Parameter, name value pairs of each parameter given.
        return config_list


    def execute_bb_adaptation(self, param_msg): 

        req_bb_exec = SetParameterInBlackboard.Request()

        self.get_logger().info("\n\n\nparam type \n\n\n" + str(param_msg.value.type))
        if type(param_msg) is not list:
            param_msg = [param_msg]



        for par in param_msg:
            self.reporting_dict[par.name] = value_from_param(par)

        req_bb_exec.ros_parameters = param_msg

        

        while not self.cli_bb_exec.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set param in bb service not available, waiting again...')
        
        response = self.cli_bb_exec.call(req_bb_exec)    

        return response.success



    def create_set_param_client(self, node_name):
        self.set_parameter_client_dict[node_name] = self.create_client(SetParameters, '/' + node_name + '/set_parameters', callback_group=MutuallyExclusiveCallbackGroup())

    def create_change_state_client(self, node_name):
        self.change_state_client_dict[node_name] = self.create_client(ChangeState, '/' + node_name + '/change_state', callback_group=MutuallyExclusiveCallbackGroup())


    def execute_rp_adaptation(self, param_msg, node_name):
        if node_name not in self.set_parameter_client_dict: 
            self.create_set_param_client(node_name)  

        client = self.set_parameter_client_dict[node_name]
        

        if type(param_msg) is not list:
            param_msg = [param_msg]

        for par in param_msg:
            self.reporting_dict[par.name] = value_from_param(par)

        req_rp_exec = SetParameters.Request()



        req_rp_exec.parameters = param_msg


        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_param service not available, waiting again...')
            
        response = client.call(req_rp_exec)
        
        if(not all([res.successful for res in response.results])):
            self.get_logger().warning('One or more requests to set a parameter were unsuccessful in the Adaptation Manager, see reason(s):' + str(response.results))
            return False
        
        self.get_logger().info('ros param adaptation complete.')
        return True
    
    def execute_lc_adaptation(self, transition, node_name):
        self.get_logger().info("\n\n\ LC adaptation \n\n\n\n\n")

        
        if node_name not in self.change_state_client_dict: 
            self.create_change_state_client(node_name)

        client = self.change_state_client_dict[node_name]


        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('change_state service not available, waiting again...')
        
        self.get_logger().info("\n\n\Sending LC adaptation request\n\n\n\n\n")


        req_lc_exec = ChangeState.Request()
        req_lc_exec.transition = transition

        response = client.call(req_lc_exec)

        if(not response.success):
            self.get_logger().warning('A request to change a state was unsuccessful in the Adaptation Manager')
        

        self.get_logger().info("\n\n\nFinishing LC adaptation\n\n\n\n\n")

        return response.success
            



    # def timer_callback(self):        
    #     #I should probably make functions like these into utilities, like as members of a subclass of Node..        
    #     msg = AdaptationState()
    #     msg.qr_values = self.get_system_utility()
    #     msg.system_possible_configurations = self.get_system_vars()
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('\n\n\nPublishing: "%s"\n\n\n\n\n' % msg)
    #     self.i += 1


    def offline_adaptation_requested(self, request, response):
        # rebet_msgs/Adaptation[] adaptation
        # ---
        # bool success
        self.get_logger().info('\n\n offline adaptation \n\n')
        
        self.get_logger().info(str(request.adaptations))

        self.report_on_system([])
        for adaptation_to_execute in request.adaptations:
            adaptation_results = []
            target_of_adaptation = adaptation_to_execute.adaptation_target

            if(target_of_adaptation is None): 
                self.get_logger().error("Unknown or unspecified type of adaptation")
                response.success = False
                return response


            if(target_of_adaptation == Adaptation.STATETRANSITION):
                self.get_logger().info('\n\n ros_lc adaptation \n\n')
                is_exec_success = self.execute_lc_adaptation(adaptation_to_execute.lifecycle_adaptation,adaptation_to_execute.node_name)                
            elif(target_of_adaptation == Adaptation.ROSPARAMETER):
                self.get_logger().info('\n\n ros_param adaptation \n\n')
                is_exec_success = self.execute_rp_adaptation(adaptation_to_execute.parameter_adaptation,adaptation_to_execute.node_name)
            elif(target_of_adaptation == Adaptation.BLACKBOARDENTRY):
                self.get_logger().info('\n\n blackboard adaptation \n\n')
                is_exec_success = self.execute_bb_adaptation(adaptation_to_execute.blackboard_adaptation)

            adaptation_results.append(is_exec_success)


        response.success = all(adaptation_results)
        return response 
       
        
    def online_adaptation_requested(self, request, response):
        # rebet_msgs/AdaptationOptions[] adaptation_space
        # string task_identifier
        # string adaptation_strategy
        # ---
        # bool success

        task_identifier = str(request.task_identifier)
        adaptation_strategy = str(request.adaptation_strategy)

        response.success = False
        adapt_state = AdaptationState()
        
        adaptation_at_system_level = "system" in task_identifier #TODO: make this more robust.
            

        adapt_state.qr_values = self.get_system_utility(adaptation_at_system_level)

        self.report_on_system(adapt_state.qr_values)
        self.get_logger().info('\n\n sys util \n\n')

        adapt_state.system_possible_configurations = self.make_configurations(request.adaptation_space)

        self.get_logger().info(str(adapt_state.system_possible_configurations))
        


        
        # all_the_qrs = []

        # for i in range(5):
        #     qr_val = QRValue()
        #     qr_val.name = "test"
        #     qr_val.qr_fulfilment = 5.0
        #     all_the_qrs.append(qr_val)

        # adapt_state.qr_values = all_the_qrs

        #new task or new strategy for the same task.

        if ( (task_identifier not in self.task_to_strategy_map) or ( (task_identifier in self.task_to_strategy_map) and (adaptation_strategy != self.task_to_strategy_map[task_identifier].get_name()) ) ):
            self.task_to_strategy_map[task_identifier] = create_strategy(adaptation_strategy)
        elif(request.adaptation_strategy == "reset"):
            #reset of same strategy during task.
            self.get_logger().info('\n\n RESET OF STRATEGY \n\n')

            self.task_to_strategy_map[task_identifier] = create_strategy(self.task_to_strategy_map[task_identifier].get_name())
        else:
            self.get_logger().info('\n\n REUSE OF STRATEGY FOR TASK:' + task_identifier+'\n\n')



        
        suggested_configuration = self.task_to_strategy_map[task_identifier].suggest_adaptation(adapt_state)
        #Configuration.msg
        #string[] node_names
        #int8[] adaptation_target_types
        #rcl_interfaces/Parameter[] configuration_parameters
        
        for i in range(len(suggested_configuration.node_names)):
            node_name = suggested_configuration.node_names[i]
            type_of_adaptation = suggested_configuration.adaptation_target_types[i]
            adaption_param = suggested_configuration.configuration_parameters[i]

            adaptation_responses = []
             
            if(type_of_adaptation == Adaptation.ROSPARAMETER):
                self.get_logger().info('\n\n ros_param adaptation \n\n')
                is_exec_success = self.execute_rp_adaptation(adaption_param,node_name)
            elif(type_of_adaptation == Adaptation.BLACKBOARDENTRY):
                self.get_logger().info('\n\n blackboard adaptation \n\n')
                is_exec_success = self.execute_bb_adaptation(adaption_param)
            else:
                self.get_logger().error("Unknown or unspecified adaptation_target")
                response.success = False
                return response
            adaptation_responses.append(is_exec_success)

        response.success = all(adaptation_responses)
        
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