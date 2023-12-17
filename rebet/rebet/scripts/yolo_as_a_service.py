#!/usr/bin/env python3

from ultralytics import YOLO
from cv_bridge import CvBridge
from rebet_msgs.srv import DetectObject
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory



class YoloAsAService(Node):

    def __init__(self):
        super().__init__('detect_object')
        self.srv = self.create_service(DetectObject, 'detect_object_srv', self.detect_obj_service)
        
        self.bridge = CvBridge()

        package_name = "rebet"
        weight_dir = get_package_share_directory(package_name) + "/config/" + "yolov8n.pt" 

        self.model = YOLO(weight_dir)
        self.get_logger().info('created')


    def detect_obj_service(self, request, response):
        self.get_logger().info('Incoming request')


        rgb_msg = request.image

        im = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")

        results = self.model.predict(source=im, save=False, save_txt=False)  # save predictions as labels

        response.labels = []
        response.probabilities = []
        for i in range(len(results)):
            try:
                response.labels.append(str(results[i].names[int(results[i].boxes.cls.cpu().numpy()[i])]))
                response.probabilities.append(float(results[i].boxes.conf.cpu().numpy()[i]))
            except IndexError:
                continue

        return response


def main():
    rclpy.init()

    minimal_service = YoloAsAService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()