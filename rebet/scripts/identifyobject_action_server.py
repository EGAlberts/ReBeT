#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node

from rebet_msgs.action import IdentifyObject
from std_msgs.msg import Float64
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from nav2_msgs.action import NavigateToPose

from darknet_ros_msgs.action import CheckForObjects
from copy import deepcopy
from itertools import cycle
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import math


PICTURE_RT_PARAM = "pic_rate"
GOAL_OBJ_NAME_PARAM = "goal_obj"
DET_THRESH_PARAM = "det_threshold"
              
class IdentifyObjectActionServer(Node):

    def __init__(self):
        super().__init__('identify_action_server')
        self._action_server = ActionServer(
            self,
            IdentifyObject,
            'identify_object',
            self.execute_callback,
            callback_group=MutuallyExclusiveCallbackGroup())
        
        self.declare_parameter(GOAL_OBJ_NAME_PARAM, "fire hydrant")
        self.declare_parameter(PICTURE_RT_PARAM, 1)
        self.declare_parameter(DET_THRESH_PARAM,14)
        

        self.check_obj_acclient = ActionClient(self, CheckForObjects, 'checkForObjectsActionName', callback_group = MutuallyExclusiveCallbackGroup())


        self.subscription = self.create_subscription(Image,'/camera/image_raw',self.tb_image_cb,10, callback_group = MutuallyExclusiveCallbackGroup())
        
        self.tb_image = None

        self.counter = 0
        self.prev_done = True
        self.ID_fdback_msg = Identify.Feedback()
        self.ID_fdback_msg.obj_idd.object_names = []
        self.ID_fdback_msg.obj_idd.probabilities = []
        self.ID_fdback_msg.obj_idd.object_detected = False
        self.initial_time = time.time()
        self.times_detected = 0
        #bool object_detected
        #string object_name
        #float32 probability
        self.get_logger().info('ID Action server created...')

        self.picture_rate = self.get_parameter(PICTURE_RT_PARAM).get_parameter_value().integer_value
        self.ID_fdback_msg.picture_rate = self.picture_rate

        self.detection_threshold = self.get_parameter(DET_THRESH_PARAM).get_parameter_value().integer_value #parameterize
        self.goal_object = self.get_parameter(GOAL_OBJ_NAME_PARAM).get_parameter_value().string_value

 
    def tb_image_cb(self,msg):
        self.tb_image = msg
        self.counter+=1
        if(self.counter % 1000 == 0): self.get_logger().info('1000th Image Msg Received!!')
        if self.counter > 200000: self.counter = 0


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        self.get_logger().info('Its me, hi, Im a ID action server')
        
        self.picture_taken = False
  
        self.picture_rate = self.get_parameter(PICTURE_RT_PARAM).get_parameter_value().integer_value
        self.get_logger().info("Picture Rate: " + str(self.picture_rate))
        
        for i in range(self.picture_rate):
            #take a picture

            #take new
            self.prev_done = False
            self.send_goal()
            
            self.ID_fdback_msg.obj_idd.stamp = self.get_clock().now().to_msg()
            goal_handle.publish_feedback(self.ID_fdback_msg)
            

        goal_handle.succeed()
        res = Identify.Result()
        res.time_elapsed = time.time() - self.initial_time
        res.picture_rate = self.picture_rate
        return res



    def send_goal(self):
        #ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 1.9, y: 0.7, z: 0.0}, orientation: {w: 1.0}}}"

        self.get_logger().info("Sending the goal")

        """Send a `CheckForObjects` action request."""
        self.get_logger().debug("Waiting for 'CheckForObjects' action server")
        while not self.check_obj_acclient.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'CheckForObjects' action server was not available, will try again")
            

        goal_msg = CheckForObjects.Goal()
        goal_msg.id = 1
        while self.tb_image is None:
            self.get_logger().info("No image received yet to send, waiting...")
        #self.get_logger().info("Uhh:" + str(type(self.tb_image)))

        goal_msg.image = self.tb_image

        send_goal_future = self.check_obj_acclient.send_goal_async(goal_msg, self._feedbackCallback)
        
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future,timeout_sec=20)
        self.ID_fdback_msg.obj_idd.object_names = []
        self.ID_fdback_msg.obj_idd.probabilities = []
        self.ID_fdback_msg.obj_idd.object_detected = False
        result = get_result_future.result().result
        if(len(result.bounding_boxes.bounding_boxes) > 0):
            self.ID_fdback_msg.obj_idd.object_detected = True
            if(self.goal_object in [box.class_id for box in result.bounding_boxes.bounding_boxes]):
                self.times_detected += 1
            for box_index, box in enumerate(result.bounding_boxes.bounding_boxes):
                self.ID_fdback_msg.obj_idd.object_names.append(box.class_id)
                self.ID_fdback_msg.obj_idd.probabilities.append(box.probability)
                self.get_logger().info('Box: {2} Result: {0} Probability: {1}'.format(box.class_id, str(box.probability), box_index))
                self.get_logger().info('Times Detected: {0}'.format(self.times_detected))
        else:
                self.get_logger().info('Nothing detected')
                self.get_logger().info('Times {1} Detected: {0}'.format(self.times_detected, self.goal_object))


        self.prev_done = True


        return True


    def _feedbackCallback(self, msg):
        self.get_logger().debug('Received action feedback message')
        self.feedback = msg.feedback
        return
    
def main(args=None):
    rclpy.init(args=args)

    id_action_server = IdentifyObjectActionServer()
    
    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(id_action_server)
    mt_executor.spin()

    id_action_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()