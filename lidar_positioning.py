import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
import tf2_ros
import numpy as np
from pyswarms.single.global_best import GlobalBestPSO
from queue import Queue
from time import sleep
import threading
from message_filters import ApproximateTimeSynchronizer, Subscriber
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import PointCloud2
from scipy.spatial.transform import Rotation
# self.lidar_list = lidar_list
#         self.lidar_frames, self.lidar_pcl, self.lidar_times = {}, {}, {}
#         self.det_model = det_model
#         self.tf_sub = self.create_subscription(TFMessage, '/tf', self.read_lidar_frames, 20)
#         self.lidar_sub = {}
#         for lidar in lidar_list:
#             self.lidar_sub[lidar] = Subscriber(self, PointCloud2, lidar)
#         self.tf_frames_event = threading.Event()
#         self.detection_thread = threading.Thread(target=self.detection_loop)
#         self.curr_lidar = queue.Queue(maxsize=1)
#         self.pub_cls = self.create_publisher(PointCloud2, '/lidar_clustering', 10)
#         self.human_seg_pub = self.create_publisher(PointCloud2, '/human_seg', 10)
#         self.human_dot_pub = self.create_publisher(PointCloud2, '/human_detections', 10)
#         self.det_thresh = 0.8
        
#     def read_lidar_frames(self, msg):
#         tf_mat = self.tfmsg_to_matrix(msg)
#         if (msg.transforms[0].child_frame_id not in list(self.lidar_frames.keys())) and \
#             (msg.transforms[0].child_frame_id in self.lidar_list):
#             self.lidar_frames[msg.transforms[0].child_frame_id] = tf_mat
#         if len(list(self.lidar_frames.keys())) == len(self.lidar_list):
#             self.tf_frames_event.set()
    
#     def read_lidar(self, *lidar_N):
#         lidar_len = len(self.lidar_list)
#         pcl_time = 0
#         pcl_arrays = [self.pcl2_to_numpy(lidar_N[i], self.lidar_frames[lidar_name]) \
#                       for i, lidar_name in enumerate(self.lidar_list)]
#         total_pcl = np.concatenate(pcl_arrays, axis=0)
#         for lidar_i, lidar in enumerate(self.lidar_list):
#             pcl_time += lidar_N[lidar_i].header.stamp.sec + lidar_N[lidar_i].header.stamp.nanosec * 1e-9
#         if self.curr_lidar.full(): self.curr_lidar.get_nowait()
#         self.curr_lidar.put({'data': total_pcl, 'time': pcl_time / len(self.lidar_list)})
    
#     def tfmsg_to_matrix(self, tf_msg):            # move to utils
#         # Extract the transform information from the message
#         transform = tf_msg.transforms[0].transform
#         translation = transform.translation
#         rotation = transform.rotation
#         translation_array = np.array([translation.x, translation.y, translation.z])
#         rotation_array = np.array([rotation.x, rotation.y, rotation.z, rotation.w])
#         rotation_matrix = Rotation.from_quat(rotation_array).as_matrix()[:3, :3]
#         translation_matrix = np.eye(4)
#         translation_matrix[:3, 3], translation_matrix[:3, :3], translation_matrix[3, 3] = translation_array, \
#                                                                             rotation_matrix, 1
#         return translation_matrix

#     def pcl2_to_numpy(self, msg, tf):
#         pc = ros2_numpy.numpify(msg)
#         points= pc['xyz']
#         points = points[np.all(np.isfinite(points), axis=1)]
#         points = np.dot(points, tf[:3, :3].T) + tf[:3, 3].T
#         return points

class PositioningNode(Node):
    def __init__(self, lidars):
        super().__init__('positioning_node')
        self.publisher_ = self.create_publisher(TFMessage, '/tf', 10)
        self.timer_ = self.create_timer(0.01, self.publish_tf)
        self.lidars = lidars
        self.newLidars = Queue(maxsize=1)
        t = threading.Thread(target=lambda:self.find_optimal_pos([-4, -4], [4, 4]))
        t.start()
        self.last_update = 0

    def publish_tf(self):
        if(self.newLidars.full()):
            self.lidars = self.newLidars.get()
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
            self.last_update = static_transform.header.stamp.sec + static_transform.header.stamp.nanosec * 1e-9
    
    def optim_func(self, poses):
        lidars = {}
        for i, pose in enumerate(poses):
            lidars[f'lidar_{i + 1}'] = {'x': pose[0], 'y': pose[1], 'z':self.lidars[f'lidar_{i + 1}']['z']}
        self.newLidars.put(lidars)
        while(self.newLidars.full()): sleep(0.001)
        sleep(0.1)
        print(self.last_update)
        return 0

    def find_optimal_pos(self, lb, ub):
        options = {'c1': 0.5, 'c2': 0.3, 'w': 0.9}
        optimizer = GlobalBestPSO(n_particles=len(list(self.lidars.keys())), dimensions=2, options=options, bounds=(lb, ub))
        x_opt, f_opt = optimizer.optimize(self.optim_func, 1000)
        print(x_opt, f_opt)
    
def main():
    lidars = {'lidar_1': {'x':3, 'y':-3, 'z': 0.6}, 'lidar_2': {'x':2, 'y':0, 'z': 0.7}}
    rclpy.init()
    node = PositioningNode(lidars)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()