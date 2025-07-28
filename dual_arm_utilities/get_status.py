from rclpy.node import Node
import ast  # Safely evaluate string representation of list
import darm_msgs.msg
from dual_arm_utilities.utilities import create_csv
from datetime import datetime


class GetStatus(Node):
    def __init__(self):
        super().__init__('my_node')

        self.declare_parameter('status', 'default')
        self.declare_parameter('status_list', ['idle', 'running'])

        self.status = self.get_parameter('status').get_parameter_value().string_value
        status_list_str = self.get_parameter('status_list').get_parameter_value().string_value

        # Convert the string list to an actual list
        try:
            self.status_list = ast.literal_eval(status_list_str)
        except Exception as e:
            self.get_logger().error(f'Failed to parse status_list: {e}')
            self.status_list = []

        self.get_logger().info(f'Status: {self.status}')
        self.get_logger().info(f'Status list: {self.status_list}')

        self.cmd_status_f = False
        self.warn_once_f = False
        self.file_path_fk = '/home/kd/Documents/dual_arm_ws/src/dual_arm_utilities/data/fil1.csv'
        self.file_path_desired = '/home/cstar/Documents/dual_arm_ws/src/end_effector_tracking/data/desired_ee_pose_1.csv'
        self.file_path_aruco = '/home/cstar/Documents/dual_arm_ws/src/end_effector_tracking/data/aruco_ee_pose_1.csv'
        create_csv(self.file_path_fk)
        self.create_csv(self.file_path_desired)
        self.create_csv(self.file_path_aruco)

        self.pub = self.create_subscription(darm_msgs.msg.UiCommand,"svaya/ui/command", self.cmd_callback,10)
        self.sub = self.create_subscription(darm_msgs.msg.UiStatus, "svaya/ui/status",self.js_callback,10)

    
    def js_callback(self, msg):
        if self.cmd_status_f:
            if self.warn_once_f:
                self.get_logger().info(f"Received cmd")
                self.warn_once_f = False
            self.status = msg
            self.joint_state_actual = np.concatenate((self.status.left_arm.position, self.status.right_arm.position))
            self.joint_callback_status  = True
        else:
            if not self.warn_once:
                self.get_logger().info(f"Command to robot has'nt been published yet...")
                self.warn_once_f = True
    
    def cmd_callback(self, msg):
        self.cmd_status  = True
    