# Used to determine which of the next *_value fields are set.

# SystemAttributeValue.PARAMETER_NOT_SET indicates that the parameter was not set

# (if gotten) or is uninitialized.

# Values are enumerated in `SystemAttributeType.msg`.

std_msgs/Header header

# The type of this parameter, which corresponds to the appropriate field below.

uint8 type

# "Variant" style storage of the SystemAttribute value. Only the value corresponding

# the type field will have valid information. This is inspired by the Parameter messages in rcl_interfaces.


# An Odometry value which represents the motion of a robot

nav_msgs/Odometry odom_value

diagnostic_msgs/KeyValue diag_value

sensor_msgs/LaserScan laser_value

std_msgs/Float32 float_value
