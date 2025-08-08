from rclpy.node import Node
import rclpy

from std_msgs.msg import Bool, Float64MultiArray
from fts_msgs.msg import FtsData
import darm_msgs.msg

from dual_arm_utilities.utilities import create_csv
import numpy as np
import csv

class GetStatus(Node):
    def __init__(self):
        super().__init__('status_node')
        self.do_not_modify_init()
    
    def do_not_modify_init(self):
        self.declare_parameter('sampling_frequency', 100.0)
        self.declare_parameter('position', False)
        self.declare_parameter('velocity', False)
        self.declare_parameter('torque', False)
        self.declare_parameter('ft', False)
        self.declare_parameter('base_path', '/home/svaya/Desktop/rde_darm_testing/src/dual_arm_utilities/data/')
        self.declare_parameter('file_name', 'exp0')
        self.declare_parameter('data_at_sampling_frequency', True)
        self.declare_parameter('pose', True)
        self.sampling_frequency = self.get_parameter('sampling_frequency').get_parameter_value().double_value
        self.position = self.get_parameter('position').get_parameter_value().bool_value
        self.velocity = self.get_parameter('velocity').get_parameter_value().bool_value
        self.torque = self.get_parameter('torque').get_parameter_value().bool_value
        self.ft = self.get_parameter('ft').get_parameter_value().bool_value
        self.file_name = self.get_parameter('file_name').get_parameter_value().string_value
        self.base_path = self.get_parameter('base_path').get_parameter_value().string_value
        self.data_at_sampling_frequency = self.get_parameter('data_at_sampling_frequency').get_parameter_value().bool_value
        self.pose = self.get_parameter('pose').get_parameter_value().bool_value
        self.counter = 0.0

        self.traj_status = False
        self.warn_once_f = False

        self.timer = self.create_timer(1/self.sampling_frequency, self.timer_callback)
        
        self.file_paths = self.file_path_create_init(position=self.position, velocity=self.velocity, torque=self.torque, ft=self.ft)

        self.create_subscription(darm_msgs.msg.UiStatus, "svaya/ui/status",self.status_calback,10)
        self.create_subscription(Float64MultiArray, 'traj/pose', self.pose_callback, 10)
        self.create_subscription(FtsData, "/svaya/fts/status",self.fts_callback,10)
        self.create_subscription(Bool, "traj/status",self.traj_status_calback,10)
        self.get_logger().info(f"Initialized parameters:\n sampling_frequency:{self.sampling_frequency}")
        if self.pose:
             self.pose_path = create_csv(self.base_path,data=f'pose_{self.file_name}')
       
    def file_path_create_init(self, position, velocity, torque, ft):
        paths = []
        if position:
            pfp = create_csv(self.base_path,data=f'position_{self.file_name}')
            paths.append(pfp)
            self.get_logger().info(f"Creating csv file for position:{pfp}")
        if velocity:
            vfp = create_csv(self.base_path, data=f'velocity_{self.file_name}')
            paths.append(vfp)
            self.get_logger().info(f"Creating csv file for velocity:{vfp}")
        if torque:
            tfp = create_csv(self.base_path, data=f'torque_{self.file_name}')
            paths.append(tfp)
            self.get_logger().info(f"Creating csv file for torque:{tfp}")
        if ft:
            ftfp = create_csv(self.base_path, data=f'ft_{self.file_name}')
            paths.append(ftfp)
            self.get_logger().info(f"Creating csv file for ft:{ftfp}")
        return paths
    
    def fts_callback(self, msg):
        if not self.data_at_sampling_frequency:
            if self.traj_status:
                if self.ft:
                    with open(self.file_paths[-1], mode='a', newline='') as file:
                            writer = csv.writer(file)
                            writer.writerow(np.concatenate(([self.counter], msg.data)))
    
    def status_calback(self, msg):
        if not self.data_at_sampling_frequency and (self.position or self.velocity or self.torque or self.ft):
            if self.traj_status:
                if self.warn_once_f:
                    self.get_logger().info(f"Received cmd")
                    self.warn_once_f = False
                data_lit = []
                self.status = msg
                if self.position:
                    dposition = np.concatenate(([self.counter],self.status.left_arm.position, self.status.right_arm.position))
                    data_lit.append(dposition)
                if self.velocity:
                    dvelocity = np.concatenate(([self.counter],self.status.left_arm.velocity, self.status.right_arm.velocity))
                    data_lit.append(dvelocity)
                if self.torque:
                    dtorque = np.concatenate(([self.counter],self.status.left_arm.torque, self.status.right_arm.torque))
                    data_lit.append(dtorque)
                for i in range(len(data_lit)):
                    with open(self.file_paths[i], mode='a', newline='') as file:
                        writer = csv.writer(file)
                        writer.writerow(data_lit[i])
                self.counter += 1/self.sampling_frequency
            else:
                if not self.warn_once_f:
                    self.get_logger().info(f"No command to robot...")
                    self.warn_once_f = True
               
        else:
            if self.position or self.velocity or self.torque or self.ft:
                self.status = msg
    
    def pose_callback(self, msg):
        if self.traj_status:
            if self.warn_once_f:
                self.get_logger().info(f"Received cmd")
                self.warn_once_f = False
            if self.pose:
                pose = msg.data
            with open(self.pose_path, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(pose)
        else:
            if not self.traj_status and not self.warn_once_f:
                self.get_logger().info(f"Saved pose data at: {self.pose_path}")
            if not self.warn_once_f:
                self.get_logger().info(f"No command to robot...")
                self.warn_once_f = True

   
    def traj_status_calback(self, msg):
        self.traj_status = msg.data
    
    def timer_callback(self):
        if self.data_at_sampling_frequency:
            if self.traj_status:
                if self.warn_once_f:
                    self.get_logger().info(f"Received cmd")
                    self.warn_once_f = False
                data_lit = []
                if self.position:
                    dposition = np.concatenate(([self.counter],self.status.left_arm.position, self.status.right_arm.position))
                    data_lit.append(dposition)
                if self.velocity:
                    dvelocity = np.concatenate(([self.counter],self.status.left_arm.velocity, self.status.right_arm.velocity))
                    data_lit.append(dvelocity)
                if self.torque:
                    dtorque = np.concatenate(([self.counter],self.status.left_arm.torque, self.status.right_arm.torque))
                    data_lit.append(dtorque)
                for i in range(len(data_lit)):
                    with open(self.file_paths[i], mode='a', newline='') as file:
                        writer = csv.writer(file)
                        writer.writerow(data_lit[i])
                self.counter += 1/self.sampling_frequency
              

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