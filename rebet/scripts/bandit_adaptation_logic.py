#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from masced_bandits.bandit_options import initialize_arguments, bandit_args
from masced_bandits.bandits import init_bandit
from masced_bandits.utilities import truncate

from rebet_msgs.msg import AdaptationState
from std_msgs.msg import Float64
import sys
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
import csv
import os
import numpy as np
from rebet.adaptation_logic import AdaptationLogic

HYPERPARAM_PARAM = "hyperparameters"
BANDITNAME_PARAM = "bandit_name"

session_names = [
    "Quantum",
    "Nebula",
    "Odyssey",
    "Cipher",
    "Zenith",
    "Synapse",
    "Apex",
    "Cosmos",
    "Genesis",
    "Spectrum",
    "Infinity",
    "Aegis",
    "Velocity",
    "Orion",
    "Phoenix",
    "Catalyst",
    "Elysium",
    "Helix",
    "Serenity",
    "Pinnacle",
    "Eclipse",
    "Pandora",
    "Polaris",
    "Nova",
    "Aether",
    "Metropolis",
    "Aurora",
    "Equinox",
    "Radiance",
    "Celestial",
    "Paragon",
    "Quasar",
    "Epoch",
    "Luminary",
    "QuantumCore",
    "Nebulous",
    "Ascendant",
    "Zenithal",
    "Heliosphere",
    "Astrolabe",
    "Empyrean",
    "Utopia",
    "Ethereal",
    "Synthwave",
    "Solstice",
    "Stardust",
    "Apexia",
    "Temporal",
    "AstraNova",
    "Perigee"
]




chosen_arms = []





class BanditAdaptationLogic(AdaptationLogic):

    def __init__(self):
        super().__init__('bandit_adaptation_logic')
        
        self.declare_parameter(HYPERPARAM_PARAM,[""])
        self.hyper_parameters = self.get_parameter(HYPERPARAM_PARAM).get_parameter_value().string_array_value
        self.hyper_param_kwargs = {}

        if self.hyper_parameters != [] and self.hyper_parameters != [""]:
            for i in range(0,len(self.hyper_parameters),2):
                self.hyper_param_kwargs[self.hyper_parameters[i]] = self.hyper_parameters[i+1]
        
        self.declare_parameter(BANDITNAME_PARAM,"UCB")
        self.bandit_name = self.get_parameter(BANDITNAME_PARAM).get_parameter_value().string_value

        self.get_logger().info("bandit name " + self.bandit_name)

        self.session_name = np.random.choice(session_names)
        # self.cli = self.create_client(SetParameters, '/identify_action_server/set_parameters', callback_group=MutuallyExclusiveCallbackGroup())
        self.req = SetParameters.Request()
        
        csv_file = 'bandit_report.csv'
        file = open(csv_file, mode='a', newline='')
        self.writer = csv.writer(file)

        file_exists = os.path.isfile(csv_file)

        if not file_exists:
            self.writer.writerow(['Session_Name', 'Timestamp', 'Adaptation Chosen', 'Reward', 'Bounds']) 

        file.close()
        bandit_args["dynamic_bounds"] = True
        bandit_args["bounds"] = [0,0.000000000001]

        print(bandit_args["bounds"])

        self.configuration_dict = {}
        self.prev_configurations_msg = None
        self.arm_change_flag = False
        self.get_logger().info('Action server created...')
        self.first_msg_received = False
        self.qr_values = []
        self.set_parameter_client_dict = {}
        self.bandit_not_initialized = True


    def create_set_param_client(self, node_name):
        self.set_parameter_client_dict[node_name] = self.create_client(SetParameters, '/' + node_name + '/set_parameters', callback_group=MutuallyExclusiveCallbackGroup())

    def set_parameters_of_node(self, config_msg):
        send_dict = {}
        node_names = config_msg.node_names
        params = config_msg.configuration_parameters
        
        for i in range(len(node_names)):
            curr_node_name = node_names[i]
            if curr_node_name not in send_dict: send_dict[curr_node_name] = [params[i]]
            else:
                send_dict[curr_node_name].append(params[i])

        #send requests

        self.get_logger().info('Send dict: !! ' + str(send_dict))
        
        for node_name in list(send_dict.keys()):
            self.req.parameters = send_dict[node_name]

            if node_name not in self.set_parameter_client_dict: 
                self.create_set_param_client(node_name)

            client = self.set_parameter_client_dict[node_name]

            while not client.wait_for_service(timeout_sec=1.0):
                        self.get_logger().info('set_param service not available, waiting again...')
            
            response = client.call(self.req)
            
            if(not all([res.successful for res in response.results])):
                self.get_logger().warning('One or more requests to set a parameter were unsuccessful in the Bandit, see reason(s):' + str(response.results))
        
    



    def do_bandit(self):
        self.get_logger().info('Executing goal...')
        
        if self.first_msg_received and self.bandit_not_initialized:
            self.get_logger().info('Bandit action server is waiting for an initial message ' + str(bandit_args["bounds"]),throttle_duration_sec=5)
            self.get_logger().info("Arms it was given: " + str(list(self.configuration_dict.keys())))

            bandit_args["arms"] = list(self.configuration_dict.keys())
            bandit_args["initial_configuration"] = bandit_args["arms"][0]
            self.bandit_instance = init_bandit(name=self.bandit_name, **self.hyper_param_kwargs)
            # self.get_logger().info("epsi " + str(self.bandit_instance.epsilon))

            self.bandit_not_initialized = False

        elif(self.first_msg_received == False): return #safety
        

        self.get_logger().info(str(list(self.configuration_dict.keys())))
        self.get_logger().info(str(list(self.configuration_dict)))


        

        #since python 3.7 the order given by .keys() is guaranteed to be insertion order, this means any new arms should be added added to a len+1 index in the dictionary and not get confused for others.

        #initialize_arguments(list(self.configuration_dict.keys()), 0, bounds = (0,0.0001), dynamic_bounds=True) #this 0 is an assumption that can only be resolved by giving the initial configuration to this node somehow.

        if(self.arm_change_flag):
            self.get_logger().info("Arm changes")

            self.bandit_instance.arms = list(self.configuration_dict.keys()) #The chosen bandit might not properly support the change in arms.
            self.arm_change_flag = False

        next_arm = self.bandit_instance.get_next_arm(self.reward)

        self.get_logger().info("Bandit adapting to " + str(next_arm))

        file = open('bandit_report.csv', mode='a', newline='')
        writer = csv.writer(file)
        row = [self.session_name, str(time.time()), str(next_arm), str(self.reward), str(bandit_args["bounds"]), str(self.qr_values), str(self.bandit_name), str(self.hyper_param_kwargs)]
        writer.writerow(row)
        file.close()
        self.get_logger().info("Wrote row " + str(row))

        
        self.set_parameters_of_node(self.configuration_dict[next_arm])
           
    def adaptation_state_callback(self, request, response):
        self.get_logger().info("bndt listener called")
        msg = request.current_state
        if(msg is not None):
            self.reward = 0 
            #for qr in msg.qr_values: self.reward+=qr.qr_fulfilment 
            self.qr_values = [qr.qr_fulfilment for qr in msg.qr_values ]
            qr_mean = np.mean(self.qr_values)
            qr_product =  np.prod(self.qr_values)
            amplifier = 2
            qr_product = qr_product + ((np.sqrt(1-qr_product) - (1 -  qr_product)) * amplifier)

            self.reward = qr_mean + qr_product
            
            self.get_logger().info('Reward and bounds before truncating ' + str(self.reward) + str(bandit_args["bounds"]))

            self.reward, _, _ = truncate(self.reward)
            
            self.possible_configs = msg.system_possible_configurations

            if(self.prev_configurations_msg != msg.system_possible_configurations):
                #New configurations have been added OR its the first time.
                # self.get_logger().info('Hashable thing happened...')

                self.make_hashable()

                self.prev_configurations_msg = msg.system_possible_configurations
                
                self.arm_change_flag = True
                
            #else:
                #self.get_logger().info('Hashable thing didnt happen...',throttle_duration_sec=10)
                

            if((self.first_msg_received is False) and (len(msg.system_possible_configurations) != 0)): 
                # self.get_logger().info('Supposedly wasnt empty..'+ str(msg.system_possible_configurations))
                
                
                
                self.first_msg_received = True
            self.do_bandit()
        
        response.success = True
        return response


    def make_hashable(self):
        for config_msg in self.possible_configs:
            string_repr = " ".join([str((param.name, param.value)) for param in config_msg.configuration_parameters] + config_msg.node_names)
            if string_repr not in self.configuration_dict:
                self.configuration_dict[string_repr] = config_msg
        





def main(args=None):
    rclpy.init(args=args)

    bandit_adaptation_logic = BanditAdaptationLogic()
    
    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(bandit_adaptation_logic)
    mt_executor.spin()

    bandit_adaptation_logic.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()