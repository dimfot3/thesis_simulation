import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import tf2_ros
import numpy as np
from queue import Queue
from time import sleep
import threading
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation
import ros2_numpy
import open3d as o3d
from skopt import gp_minimize
from skopt.space import Real



class PositioningNode(Node):
    def __init__(self, lidars, cov_bounds, pos_bounds):
        super().__init__('positioning_node')
        # declaration of tf publisher that will change position of LiDAR
        self.publisher_ = self.create_publisher(TFMessage, '/tf', 10)
        self.timer_ = self.create_timer(0.01, self.publish_tf)
        # initialize the positioning system
        self.lidars = lidars
        self.bounds = cov_bounds   
        self.eff_bounds = pos_bounds
        self.lidar_sub = {}
        self.newLidars = Queue(maxsize=1)
        self.cur_pcl = Queue(maxsize=1)
        # start the iptimization
        t = threading.Thread(target=lambda:self.find_optimal_pos())
        t.start()
        # initialize subscriber to lidars
        for lidar in self.lidars.keys():
            self.lidar_sub[lidar] = Subscriber(self, PointCloud2, lidar)
        # start lidar reading
        self.last_update = 0
        self.sync = ApproximateTimeSynchronizer([item[1] for item in self.lidar_sub.items()], queue_size=10, slop=0.4)
        self.sync.registerCallback(self.read_lidar)

    def publish_tf(self):
        if(self.newLidars.full()):
            self.lidars = self.newLidars.get()
            cur_time = self.get_clock().now().to_msg()
            self.last_update = cur_time.sec + cur_time.nanosec * 1e-9
        for lidar in self.lidars.keys():
            tf_msg = TFMessage()
            # Create the transform for the first lidar
            lidar_name = lidar
            static_transform = TransformStamped()
            static_transform.header.stamp = self.get_clock().now().to_msg()
            static_transform.header.frame_id = "world"
            static_transform.child_frame_id = lidar_name
            static_transform.transform.translation.x = float(self.lidars[lidar]["x"])
            static_transform.transform.translation.y = float(self.lidars[lidar]["y"])
            static_transform.transform.translation.z = float(self.lidars[lidar]["z"])
            static_transform.transform.rotation.x = 0.0
            static_transform.transform.rotation.y = 0.0
            static_transform.transform.rotation.z = 0.0
            static_transform.transform.rotation.w = 1.0
            tf_msg.transforms.append(static_transform)
            self.publisher_.publish(tf_msg)

    def get_lidar_frame(self):
        tr_frames = []
        for lidar in self.lidars.keys():
            translation_array = np.array([self.lidars[lidar]['x'], self.lidars[lidar]['y'], self.lidars[lidar]['z']])
            rotation_array = np.array([0, 0, 0, 1])
            rotation_matrix = Rotation.from_quat(rotation_array).as_matrix()[:3, :3]
            translation_matrix = np.eye(4)
            translation_matrix[:3, 3], translation_matrix[:3, :3], translation_matrix[3, 3] = translation_array, \
                                                                                rotation_matrix, 1
            tr_frames.append(translation_matrix)
        return tr_frames

    def read_lidar(self, *lidar_N):
        pcl_time = 0
        lidar_frames = self.get_lidar_frame()
        pcl_arrays = [self.pcl2_to_numpy(lidar_N[i], lidar_frame) \
                      for i, lidar_frame in enumerate(lidar_frames)]   
        # Keep only lidar on valid positions
        lidar_positions = np.array([frame[:2, 3] for frame in lidar_frames]).reshape(-1, 2)
        mask = np.zeros((len(lidar_frames), self.eff_bounds.shape[0]), dtype=bool)
        for i in range(len(self.eff_bounds)):
            area = self.eff_bounds[i]
            mask_area = np.logical_and(lidar_positions[:, 0] >= area[0, 0], lidar_positions[:, 0] <= area[0, 1])
            mask_area = np.logical_and(mask_area, lidar_positions[:, 1] >= area[1, 0])
            mask_area = np.logical_and(mask_area, lidar_positions[:, 1] <= area[1, 1])
            mask[:, i] = mask_area
        valid_sensor_mask = np.any(mask, axis=1)
        pcl_arrays = [array for i, array in enumerate(pcl_arrays) if valid_sensor_mask[i]]
        if(len(pcl_arrays) == 0): pcl_arrays = [np.zeros((1, 3))]
        total_pcl = np.concatenate(pcl_arrays, axis=0)
        for lidar_i in range(len(lidar_frames)):
            pcl_time += lidar_N[lidar_i].header.stamp.sec + lidar_N[lidar_i].header.stamp.nanosec * 1e-9
        if self.cur_pcl.full(): self.cur_pcl.get_nowait()
        self.cur_pcl.put({'data': total_pcl, 'time': pcl_time / len(lidar_frames)})

    def pcl2_to_numpy(self, msg, tf):
        pc = ros2_numpy.numpify(msg)
        points= pc['xyz']
        points = points[np.all(np.isfinite(points), axis=1)]
        points = np.dot(points, tf[:3, :3].T) + tf[:3, 3].T
        return points

    def pcl_voxel(self, pcl, voxel_size=0.1):
        """
        Voxelizes a point cloud based on a constant output point size.

        :param pcd: The input point cloud to voxelize.
        :param voxel_size: The size of the output points of the voxelized point cloud.
        :return: The voxelized point cloud.
        """
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(pcl)
        voxeld_pcl = np.array(pcd.voxel_down_sample(voxel_size).points)
        return voxeld_pcl

    def fitness_score(self, pcl):
        voxel_pcl = self.pcl_voxel(pcl, voxel_size=1.5)
        mask_x = np.logical_and(voxel_pcl[:, 0] >= self.bounds[0][0], voxel_pcl[:, 0] <= self.bounds[0][1])
        mask_y = np.logical_and(voxel_pcl[:, 1] >= self.bounds[1][0], voxel_pcl[:, 1] <= self.bounds[1][1])
        mask = np.logical_and(mask_x, mask_y)
        pcl_eff = voxel_pcl[mask]
        return - pcl_eff.shape[0]

    def objective_func(self, *poses):
        lidars = {}
        poses = np.array(poses).reshape(-1, 2)
        for i, pose in enumerate(poses):
            lidars[f'lidar_{i + 1}'] = {'x': pose[0], 'y': pose[1], 'z':self.lidars[f'lidar_{i + 1}']['z']}
        self.newLidars.put(lidars)
        while(self.newLidars.full()): sleep(0.01)      # wait until transform sent
        sleep(0.01)          # wait some time for the transform to happen in simulation
        while(self.cur_pcl.empty() or ((self.cur_pcl.qsize() > 0) and (self.cur_pcl.queue[0]['time'] < self.last_update))): sleep(0.01)      # wait until pcl comes
        new_pcl = self.cur_pcl.get()['data']
        score = self.fitness_score(new_pcl)
        return score

    def find_optimal_pos(self):
        space = []
        for i in range(len(self.lidars.keys())):
            space += [Real(self.bounds[0][0], self.bounds[0][1], name=f'x{i}'), Real(self.bounds[1][0], self.bounds[1][1], name=f'y{i}')]
        # run the Bayesian Optimization algorithm
        opt_res = gp_minimize(self.objective_func, space, n_calls=50, random_state=0)
        print('THISSSSS',*(opt_res.x), -opt_res.fun)
        lidars = {}
        poses = np.array([*(opt_res.x)]).reshape(-1, 2)
        for i, pose in enumerate(poses):
            lidars[f'lidar_{i + 1}'] = {'x': pose[0], 'y': pose[1], 'z':self.lidars[f'lidar_{i + 1}']['z']}
        self.newLidars.put(lidars)
    
def main():
    lidars = {'lidar_1': {'x':2, 'y':-3, 'z': 1.8}}       # lidar initialization position
    cov_bounds = np.array([[-14, 10], [-12, 7.7]])      # coverage bounds
    pos_bounds = np.array([[[-14, 10], [-12, 7.7]]])      # valid areas for lidar to be positioned
    rclpy.init()
    node = PositioningNode(lidars, cov_bounds, pos_bounds)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()