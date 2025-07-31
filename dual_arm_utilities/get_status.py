from rclpy.node import Node
import rclpy

import ast  # Safely evaluate string representation of list
from dual_arm_utilities.utilities import create_csv
from datetime import datetime
import numpy as np
import csv
import darm_msgs.msg
from std_msgs.msg import Bool



class GetStatus(Node):
    def __init__(self):
        super().__init__('status_node')

        self.declare_parameter('position', False)
        self.declare_parameter('velocity', False)
        self.declare_parameter('torque', False)
        self.declare_parameter('ft', False)
        self.position = self.get_parameter('position').get_parameter_value().bool_value
        self.velocity = self.get_parameter('velocity').get_parameter_value().bool_value
        self.torque = self.get_parameter('torque').get_parameter_value().bool_value
        self.ft = self.get_parameter('ft').get_parameter_value().bool_value

        self.traj_status = None
        self.warn_once_f = False
        
        self.file_paths = self.file_path_create_init(position=self.position, velocity=self.velocity, torque=self.torque, ft=self.ft)

        self.create_subscription(darm_msgs.msg.UiStatus, "svaya/ui/status",self.status_calback,10)
        self.create_subscription(Bool, "traj/status",self.traj_status_calback,10)

    def file_path_create_init(self, position, velocity, torque, ft):
        base_path = '/home/cstar/Documents/dual_arm_ws/src/dual_arm_utilities/data/'
        paths = []
        if position:
            pfp = create_csv(base_path,data='position')
            paths.append(pfp)
            self.get_logger().info(f"Creating csv file for position:{pfp}")
        if velocity:
            vfp = create_csv(base_path, data='velocity')
            paths.append(vfp)
            self.get_logger().info(f"Creating csv file for velocity:{vfp}")
        if torque:
            tfp = create_csv(base_path, data='torque')
            paths.append(tfp)
            self.get_logger().info(f"Creating csv file for torque:{tfp}")
        if ft:
            ftfp = create_csv(base_path, data='ft')
            paths.append(ftfp)
            self.get_logger().info(f"Creating csv file for ft:{ftfp}")
        return paths
    def status_calback(self, msg):
        if self.traj_status:
            if self.warn_once_f:
                self.get_logger().info(f"Received cmd")
                self.warn_once_f = False
            data_lit = []
            self.status = msg
            if self.position:
                dposition = np.concatenate((self.status.left_arm.position, self.status.right_arm.position))
                data_lit.append(dposition)
            if self.velocity:
                dvelocity = np.concatenate((self.status.left_arm.velocity, self.status.right_arm.velocity))
                data_lit.append(dvelocity)
            if self.torque:
                dtorque = np.concatenate((self.status.left_arm.torque, self.status.right_arm.torque))
                data_lit.append(dtorque)
            for i in range(len(self.file_paths)):
                with open(self.file_paths[i], mode='a', newline='') as file:
                    writer = csv.writer(file)
                    writer.writerow(data_lit[i])
            
        else:
            if not self.warn_once_f:
                self.get_logger().info(f"Command to robot hasn't been published yet...")
                self.warn_once_f = True
   
    def traj_status_calback(self, msg):
        self.traj_status = msg.data

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