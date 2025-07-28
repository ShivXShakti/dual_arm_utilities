import os
import csv
import matplotlib.pyplot as plt
from datetime import datetime
import time
def create_csv(self, folder_path):
    os.makedirs(folder_path, exist_ok=True)
    timestamp = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    file_name = f"pose_data_{timestamp}.csv"
    full_path = os.path.join(folder_path, file_name)
    return full_path

def plot_2d_data(self):
        fig, ((ax1, ax2, ax3), (ax4,ax5,ax6)) = plt.subplots(2,3)
        if self.desired_plot_f:
            ax1.plot(self.t, self.xdl_data)
            ax2.plot(self.t, self.ydl_data)
            ax3.plot(self.t, self.zdl_data)
            ax4.plot(self.t, self.xdr_data)
            ax5.plot(self.t, self.ydr_data)
            ax6.plot(self.t, self.zdr_data)
        if self.fk_plot_f:
            ax1.plot(self.t, self.xal_data)
            ax2.plot(self.t, self.yal_data)
            ax3.plot(self.t, self.zal_data)
            ax4.plot(self.t, self.xar_data)
            ax5.plot(self.t, self.yar_data)
            ax6.plot(self.t, self.zar_data)
        if self.aruco_plot_f:
            if len(self.xaml_data)==len(self.t):
                ax1.plot(self.t, self.xaml_data)
                ax2.plot(self.t, self.yaml_data)
                ax3.plot(self.t, self.zaml_data)
                ax4.plot(self.t, self.xaml_data)
                ax5.plot(self.t, self.yaml_data)
                ax6.plot(self.t, self.zaml_data)
        ax1.set_title("Left arm: X")
        ax2.set_title("Left arm: Y")
        ax3.set_title("Left arm: Z")
        ax4.set_title("Right arm: X")
        ax5.set_title("Right arm: Y")
        ax6.set_title("Right arm: Z")
        fig.tight_layout()
        plt.show()

def plot_3d_data(self):
        fig = plt.figure(figsize=(5,5))
        ax = fig.add_subplot(111, projection='3d')
        ax.set_title('left:3D traj')
        ax.set_xlabel('X Axis')
        ax.set_ylabel('Y Axis')
        ax.set_zlabel('Z Axis')
        if self.aruco_plot_f:
            ax.plot3D(self.xaml_data,self.yaml_data,self.zaml_data, color='green', linewidth=2)
        if self.fk_plot_f:
            ax.plot3D(self.xal_data,self.yal_data,self.zal_data, color='blue', linewidth=2)
        if self.desired_plot_f:
            ax.plot3D(self.xdl_data,self.ydl_data,self.zdl_data, color='red', linewidth=2)
        plt.show()