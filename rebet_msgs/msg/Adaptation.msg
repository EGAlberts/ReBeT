#Types of adaptations supported.
int8 STATETRANSITION=0
int8 ROSPARAMETER=1
int8 BLACKBOARDENTRY=2

int8 adaptation_type

lifecycle_msgs/Transition lifecycle_adaptation
rcl_interfaces/Parameter parameter_adaptation
#As they are practically the same, we reuse the parameter msg for blackboard entries.
rcl_interfaces/Parameter blackboard_adaptation

#In the case of parameter and lifecycle adaptation, the ros2 node they are to be applied to.
string node_name
