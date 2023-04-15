import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct 
import open3d as o3d
import matplotlib.pyplot as plt


class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('point_cloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            'lidar_1',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        #self.vis = o3d.visualization.Visualizer()
        #self.vis.create_window()
        self.saved = False

    def read_pcl(self, msg):
        # Parse the PointCloud2 message into numpy arrays for the x, y, and z fields
        # Assumes a single PointField with x, y, z fields in XYZ format
        # Does not handle other formats or multiple PointFields
        offset = msg.fields[0].offset
        point_step = msg.point_step
        x_data = []
        y_data = []
        z_data = []
        for i in range(msg.width * msg.height):
            x, y, z = struct.unpack_from('<fff', msg.data, offset)
            x_data.append(x)
            y_data.append(y)
            z_data.append(z)
            offset += point_step
        x_arr = np.array(x_data).reshape(-1, 1)
        y_arr = np.array(y_data).reshape(-1, 1)
        z_arr = np.array(z_data).reshape(-1, 1)
        pcl = np.hstack([x_arr, y_arr, z_arr])
        mask = np.any(np.isinf(pcl), axis=1)
        pcl = pcl[~mask]
        return pcl

    def listener_callback(self, msg):
        if not self.saved:
            pcl = self.read_pcl(msg)
            out_pcl = np.zeros((pcl.shape[0],4))
            out_pcl[:, :3] = pcl
            out_pcl = out_pcl.astype('float32')
            out_pcl.tofile('bedroom32.bin')
            self.saved = True
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    point_cloud_subscriber = PointCloudSubscriber()
    rclpy.spin(point_cloud_subscriber)
    # pcl = np.fromfile('./bedroom.bin', dtype='float32').reshape(-1, 4)[:, :3]
    # ax = plt.subplot(1, 1, 1, projection='3d')
    # ax.scatter(pcl[:, 0], pcl[:, 1], pcl[:, 2])
    # plt.show()

if __name__ == '__main__':
    main()