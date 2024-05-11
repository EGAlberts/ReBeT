#!/usr/bin/env python3


import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import math

DARKNESS_AMPLITUDE = 80 #half the max
DARKNESS_FREQUENCY = 0.006

class LightingChanges(Node):

   

    def __init__(self):
        super().__init__('lighting_change_node')

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.add_noise_cb, 10)
        self.pub_img = self.create_publisher(Image, '/camera/image_noisy', 10)
        self.pub_light = self.create_publisher(Float32,'/current_lighting',10)
        self.start_time = time.time()

        self.lighting_gen = self.current_lighting()
      


    def add_noise_cb(self, msg):
        im = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")


        curr_light = next(self.lighting_gen)
        noisy_image = im.astype(np.float32) - curr_light
        noisy_image = np.clip(noisy_image, 0, 255).astype(np.uint8)  # Clip pixel values to [0, 255]
        float_msg = Float32()
        float_msg.data = curr_light/(DARKNESS_AMPLITUDE*2)

        self.pub_img.publish(self.bridge.cv2_to_imgmsg(noisy_image, encoding="bgr8"))
        self.pub_light.publish(float_msg)
        
    def current_lighting(self):
        while True:
            time_since_start = time.time() - self.start_time

            yield DARKNESS_AMPLITUDE * math.sin(2 * math.pi * DARKNESS_FREQUENCY * time_since_start) + DARKNESS_AMPLITUDE

def main(args=None):
    rclpy.init(args=args)

    node = LightingChanges()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
