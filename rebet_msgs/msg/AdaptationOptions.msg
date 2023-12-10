#name of the parameter

string name

#name of the node (if any) it belongs to

string node_name

#The type of adaptation being done to the target, this is constrained choice specified in Adaptation.msg

int8 adaptation_target_type

#A presumed finite set of acceptable values for the parameter to hold.

rcl_interfaces/ParameterValue[] possible_values 
