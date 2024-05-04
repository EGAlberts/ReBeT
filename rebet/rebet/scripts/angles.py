#!/usr/bin/env python3


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Quaternion, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException

import numpy as np
import math

class FibonacciActionClient(Node):

    def quaternion_from_euler(self,ai, aj, ak):
        ai /= 2.0
        aj /= 2.0
        ak /= 2.0
        ci = math.cos(ai)
        si = math.sin(ai)
        cj = math.cos(aj)
        sj = math.sin(aj)
        ck = math.cos(ak)
        sk = math.sin(ak)
        cc = ci*ck
        cs = ci*sk
        sc = si*ck
        ss = si*sk

        q = np.empty((4, ))
        q[0] = cj*sc - sj*cs
        q[1] = cj*ss + sj*cc
        q[2] = cj*cs - sj*sc
        q[3] = cj*cc + sj*ss

        return q
    
    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

    def odom_cb(self, msg):
        self.get_logger().info('I heard: [%s]' % msg)

        goal_msg = NavigateToPose.Goal()

        goal_pose = PoseStamped()

        # goal_pose.pose.position.x = msg.pose.pose.position.x
        # goal_pose.pose.position.y = msg.pose.pose.position.y

        goal_pose.header.frame_id = "map"

        quat_tf = self.quaternion_from_euler(0.0,0.0,0.0)

# Convert a list to geometry_msgs.msg.Quaternion
        msg_quat = Quaternion(x=quat_tf[0], y=quat_tf[1], z=quat_tf[2], w=quat_tf[3])
        goal_pose.pose.orientation = msg_quat

        goal_msg.pose = goal_pose

        self._action_client.wait_for_server()

        return self._action_client.send_goal(goal_msg)




def main(args=None):
    rclpy.init(args=args)

    node = FibonacciActionClient()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
