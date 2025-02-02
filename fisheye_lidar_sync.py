#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter # To set ROS 2 parameters (use_sim_time)
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy # Manages Quality of Service (QoS) policies
from message_filters import Subscriber, ApproximateTimeSynchronizer # Provides time synchronization tools
from sensor_msgs.msg import Image, PointCloud2

class FisheyeLidarSync(Node):
    def __init__(self):
        super().__init__('fisheye_lidar_sync')

        # Enables simulated time (Binds the node’s clock to the '/clock' topic)
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # Definea a QoS that matches the publishers (Example: BEST_EFFORT for sensor data)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create subscribers with a matching QoS (can be plugged into a synchronizer)
        self.sub_cam1 = Subscriber(self, Image, '/fisheye_image_SN00012', qos_profile=sensor_qos)
        self.sub_cam2 = Subscriber(self, Image, '/fisheye_image_SN00013', qos_profile=sensor_qos)
        self.sub_cam3 = Subscriber(self, Image, '/fisheye_image_SN00014', qos_profile=sensor_qos)
        self.sub_lidar = Subscriber(self, PointCloud2, '/front_lidar/points', qos_profile=sensor_qos)

        # ApproximateTimeSynchronizer
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_cam1, self.sub_cam2, self.sub_cam3, self.sub_lidar],
            queue_size=30, # The maximum number of messages per subscriber it keeps in a buffer while trying to match timestamps
            slop=0.25  # Allows messages with timestamps within this time interval of each other to be considered synchronized
        )
        self.sync.registerCallback(self.sync_callback) # Called whenever the synchronizer finds a matching set of messages

        # Create publishers for synchronized topics
        self.pub_cam1 = self.create_publisher(Image, '/synchronized/fisheye_image_SN00012', 10)
        self.pub_cam2 = self.create_publisher(Image, '/synchronized/fisheye_image_SN00013', 10)
        self.pub_cam3 = self.create_publisher(Image, '/synchronized/fisheye_image_SN00014', 10)
        self.pub_lidar = self.create_publisher(PointCloud2, '/synchronized/front_lidar/points', 10)

        self.get_logger().info("Fisheye and LiDAR synchronization started!")

    def sync_callback(self, cam1_msg, cam2_msg, cam3_msg, lidar_msg):
        # Publish the synchronized messages
        self.pub_cam1.publish(cam1_msg)
        self.pub_cam2.publish(cam2_msg)
        self.pub_cam3.publish(cam3_msg)
        self.pub_lidar.publish(lidar_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FisheyeLidarSync()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

