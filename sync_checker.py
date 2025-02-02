#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import Image, PointCloud2

class SyncChecker(Node):
    def __init__(self):
        super().__init__('sync_checker')

        #  Enables simulated time (Binds the node’s clock to the '/clock' topic)
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        # Definea a QoS that matches the publishers (Example: BEST_EFFORT for sensor data)
        checker_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create subscribers to the synchronized topics  
        self.sub_cam1 = Subscriber(self, Image, '/synchronized/fisheye_image_SN00012', qos_profile=checker_qos)
        self.sub_cam2 = Subscriber(self, Image, '/synchronized/fisheye_image_SN00013', qos_profile=checker_qos)
        self.sub_cam3 = Subscriber(self, Image, '/synchronized/fisheye_image_SN00014', qos_profile=checker_qos)
        self.sub_lidar = Subscriber(self, PointCloud2, '/synchronized/front_lidar/points', qos_profile=checker_qos)

        # Use a small queue_size and slop since these topics should already be synced
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_cam1, self.sub_cam2, self.sub_cam3, self.sub_lidar],
            queue_size=10,
            slop=0.01  # Only allow 10ms difference
        )
        self.sync.registerCallback(self.sync_callback)

        self.get_logger().info("SyncChecker node is running to compare timestamps.")

    def sync_callback(self, cam1_msg, cam2_msg, cam3_msg, lidar_msg):
        # Each time a set of messages from all 4 synced topics arrives within 10ms, this callback calculates the differences in their 'header.stamp'.
        # A function to convert a ROS time to nanoseconds
        def to_nsec(msg):
            return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

        t1 = to_nsec(cam1_msg)
        t2 = to_nsec(cam2_msg)
        t3 = to_nsec(cam3_msg)
        t4 = to_nsec(lidar_msg)

        # Compare each to cam1's timestamp (Just an example)
        diff_cam1_cam2 = abs(t1 - t2) / 1e6  # in ms
        diff_cam1_cam3 = abs(t1 - t3) / 1e6  # in ms
        diff_cam1_lidar = abs(t1 - t4) / 1e6  # in ms

        self.get_logger().info(
            f"\nSynchronized set received!\n"
            f"  Cam1-Cam2: {diff_cam1_cam2:.4f} ms\n"
            f"  Cam1-Cam3: {diff_cam1_cam3:.4f} ms\n"
            f"  Cam1-Lidar: {diff_cam1_lidar:.4f} ms"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SyncChecker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

