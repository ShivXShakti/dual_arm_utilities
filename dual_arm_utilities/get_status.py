from rclpy.node import Node
import rclpy

import ast  # Safely evaluate string representation of list
import darm_msgs.msg
from dual_arm_utilities.utilities import create_csv
from datetime import datetime
import numpy as np
import csv


class GetStatus(Node):
    def __init__(self):
        super().__init__('status_node')

        self.declare_parameter('status', 'default')
        self.declare_parameter('status_list', ['position', 'velocity', 'torque', 'ft'])

        self.status = self.get_parameter('status').get_parameter_value().string_value
        status_list_str = self.get_parameter('status_list').get_parameter_value().string_value
        try:
            self.status_list = ast.literal_eval(status_list_str)
        except Exception as e:
            self.get_logger().error(f'Failed to parse status_list: {e}')
            self.status_list = []

        self.get_logger().info(f'Status: {self.status}')
        self.get_logger().info(f'Status list: {self.status_list}')

        self.cmd_status_f = False
        self.warn_once_f = False
        
        self.file_paths = self.file_path_create_init(position=True, velocity=True, torque=True, ft=True)

        self.pub = self.create_subscription(darm_msgs.msg.UiCommand,"svaya/ui/command", self.cmd_callback,10)
        self.sub = self.create_subscription(darm_msgs.msg.UiStatus, "svaya/ui/status",self.status_calback,10)

    def file_path_create_init(self, position, velocity, torque, ft):
        file_path_position = '/home/cstar/Documents/dual_arm_ws/src/dual_arm_utilities/data/'
        file_path_velocity = '/home/cstar/Documents/dual_arm_ws/src/dual_arm_utilities/data/v'
        file_path_torque = '/home/cstar/Documents/dual_arm_ws/src/dual_arm_utilities/data/'
        file_path_ft = '/home/cstar/Documents/dual_arm_ws/src/dual_arm_utilities/data/'
        if position:
            create_csv(file_path_position,'position')
            print(f"Creating csv file for position:{file_path_position}")
        if velocity:
            create_csv(file_path_velocity, 'velocity')
            print(f"Creating csv file for velocity:{file_path_velocity}")
        if torque:
            create_csv(file_path_torque, 'torque')
            print(f"Creating csv file for torque:{file_path_torque}")
        if ft:
            create_csv(file_path_ft, 'ft')
            print(f"Creating csv file for ft:{file_path_ft}")
        return [file_path_position, file_path_velocity, file_path_torque, file_path_ft]
    
    def status_calback(self, msg):
        if self.cmd_status_f:
            if self.warn_once_f:
                self.get_logger().info(f"Received cmd")
                self.warn_once_f = False
            self.status = msg
            position = np.concatenate((self.status.left_arm.position, self.status.right_arm.position))
            velocity = np.concatenate((self.status.left_arm.velocity, self.status.right_arm.velocity))
            torque = np.concatenate((self.status.left_arm.torque, self.status.right_arm.torque))
            with open(self.file_paths[0], mode='a', newline='') as position:
                writer = csv.writer(position)
                writer.writerow(position)
            with open(self.file_paths[1], mode='a', newline='') as velocity:
                writer = csv.writer(velocity)
                writer.writerow(velocity)
            with open(self.file_paths[2], mode='a', newline='') as torque:
                writer = csv.writer(torque)
                writer.writerow(torque)
        else:
            if not self.warn_once_f:
                self.get_logger().info(f"Command to robot hasn't been published yet...")
                self.warn_once_f = True
    
    def cmd_callback(self, msg):
        self.cmd_status_f  = True
def main(args = None):
    rclpy.init(args=args)
    node = GetStatus()
    try:
        rclpy.spin(node=node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == "__main__":
    main()