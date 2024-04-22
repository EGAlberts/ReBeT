#!/usr/bin/env python3

from ultralytics import YOLO
from cv_bridge import CvBridge
from rebet_msgs.srv import DetectObject
import rclpy
from rclpy.lifecycle import Node, State, TransitionCallbackReturn
from ament_index_python.packages import get_package_share_directory
from sensor_msgs.msg import Image
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rebet_msgs.msg import ObjectsIdentified
import time

IMG_TOPIC_PARAM = "image_topic_name"
PREDICTIONS_PARAM = "num_predictions"

class YoloAsAService(Node):

    def __init__(self):
        super().__init__('detect_object')

        
        self.bridge = CvBridge()

        package_name = "rebet"
        weight_dir = get_package_share_directory(package_name) + "/config/" + "yolov8n.pt" 

        self.model = YOLO(weight_dir)

        self.declare_parameter(IMG_TOPIC_PARAM, '/camera/image_raw')
        self.topic_name = self.get_parameter(IMG_TOPIC_PARAM).get_parameter_value().string_value

        self.declare_parameter(PREDICTIONS_PARAM, 1)

        self.image_received = None
        self.get_logger().info('created')

    def create_image_subscriber(self):
        self.subscription = self.create_subscription(
            Image,
            self.topic_name,
            self.listener_callback,
            10, callback_group = MutuallyExclusiveCallbackGroup())


    def parameter_changed_callback(self, params):
        for param in params:
            if(param.name == IMG_TOPIC_PARAM and param.value != self.topic_name):
                self.topic_name = param.value
                self.destroy_subscription(self.subscription)
                self.create_subscriber() #replace prev subscriber
                self.get_logger().info('Replaced current subscriber with subscriber to topic: "%s"' % param.value)


        return SetParametersResult(successful=True)


    def on_activate(self, state: State):
        self.srv = self.create_service(DetectObject, 'detect_object_srv', self.detect_obj_service, callback_group = MutuallyExclusiveCallbackGroup())
        
        self.create_image_subscriber()
        self.add_on_set_parameters_callback(self.parameter_changed_callback)
        
        return TransitionCallbackReturn.SUCCESS
    
    def on_deactivate(self, state: State):
        self.get_logger().info('on_deactivate() is called.')
        
        return TransitionCallbackReturn.SUCCESS if self.destroy_service(self.srv) else TransitionCallbackReturn.ERROR
    
    def listener_callback(self, msg):
        
        self.image_received = msg

    def detect_obj_service(self, request, response):
        self.get_logger().info('Incoming request')

        num_preds = self.get_parameter(PREDICTIONS_PARAM).get_parameter_value().integer_value

        for i in range(len(num_preds)):
            time.sleep(1)
            rgb_msg = self.image_received #assume to be new..

            obj_id = ObjectsIdentified()
            obj_id.object_detected = False
            if rgb_msg is not None:
                im = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
                
                results = self.model.predict(source=im, save=False, save_txt=False)  # save predictions as labels
                
                obj_id.object_names = []
                obj_id.probabilities = []
                for i in range(len(results)):
                    try:
                        obj_id.object_names.append(str(results[i].names[int(results[i].boxes.cls.cpu().numpy()[i])]))
                        obj_id.probabilities.append(float(results[i].boxes.conf.cpu().numpy()[i]))
                    except IndexError:
                        continue

                if(len(obj_id.object_names) > 0):
                    obj_id.object_detected = True

                obj_id.stamp = self.get_clock().now().to_msg()
            response.objects_id.append(obj_id)
        
        response.id = request.id
        return response
            
    


def main():
    rclpy.init()

    minimal_service = YoloAsAService()
    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(minimal_service)
    mt_executor.spin()


    rclpy.shutdown()


if __name__ == '__main__':
    main()