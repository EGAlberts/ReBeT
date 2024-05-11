#!/usr/bin/env python3


import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import csv
import time

class ReportExperiment(Node):

   

    def __init__(self):
        super().__init__('report_on_experiment_node')


        self.sub = self.create_subscription(String, '/arborist/reporting', self.update_csv, 10)
        self.rows_to_write = []
        self.count = 0

        self.csv_file = open(str(int(time.time())) + '_results.csv', 'w', newline='')
        self.get_logger().info('report node created')
        

    def write_the_rows(self, override=False):
        if((self.count % 10 == 0) or override):
            self.writer.writerows(self.rows_to_write)
            self.get_logger().info(str(self.rows_to_write))
            self.rows_to_write = []

    def update_csv(self, msg):
        self.get_logger().info('received message')
        self.count+=1
        entry_string = msg.data

        if(entry_string == "!END!"):
            self.write_the_rows(override=True)
            self.csv_file.close()
            f = open("mission.done", "w")
            f.close()
            

        header_str, values_str = entry_string.split(";")

        header_val = header_str.split(" ")
        values_val = values_str.split(" ")

        if((header_val[0] != values_val[0]) or (len(header_val) != len(values_val))):
            self.get_logger().info(str(len(header_val)))
            self.get_logger().info(str(len(values_val)))
            print(header_val)
            print(values_val)

            raise Exception("mismatch header and values somehow")
        

        header_val[0] = "time"

        if(self.count == 1):    
            self.writer = csv.DictWriter(self.csv_file,fieldnames=header_val)
            self.writer.writeheader()
        


        self.rows_to_write.append({header_val[ind] : values_val[ind] for ind in range(len(values_val))})

        self.write_the_rows()




def main(args=None):
    rclpy.init(args=args)

    node = ReportExperiment()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.write_the_rows(override=True)
        node.csv_file.close()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
