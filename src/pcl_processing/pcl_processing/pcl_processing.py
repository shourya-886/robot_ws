#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class PointCloudProcessor(Node):

    def __init__(self):
        super().__init__('pointcloud_processor')

        # Subscriber to RealSense point cloud topic
        self.subscription_ = self.create_subscription(
            PointCloud2,
            '/camera/camera/depth/color/points',
            self.process_point_cloud,
            10,
        )

        # Publisher for filtered and downsampled point cloud
        self.publisher_ = self.create_publisher(
            PointCloud2, 'filtered_pointcloud', 10
        )

    def process_point_cloud(self, msg: PointCloud2):
        # 1. Convert PointCloud2 msg to NumPy structured array
        gen = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True
        )
        points = np.array(list(gen), dtype=[('x', 'f4'), ('y', 'f4'), ('z', 'f4')])

        if len(points) == 0:
            return

        # 2. PassThrough Filtering (x: [-0.5, 0.5], y: [-0.5, 0.5], z: [0.1, 1.0])
        x = points['x']
        y = points['y']
        z = points['z']

        mask = (
            (x >= -0.5)
            & (x <= 0.5)
            & (y >= -0.5)
            & (y <= 0.5)
            & (z >= 0.1)
            & (z <= 1.0)
        )
        filtered_points = points[mask]

        if len(filtered_points) == 0:
            return

        # 3. VoxelGrid Downsampling (0.02m = 2cm leaf size)
        coords = np.vstack(
            (
                filtered_points['x'],
                filtered_points['y'],
                filtered_points['z'],
            )
        ).T
        leaf_size = 0.02

        # Assign points to discrete 3D voxel grid indices
        voxel_indices = np.floor(coords / leaf_size).astype(np.int32)
        _, unique_indices = np.unique(voxel_indices, axis=0, return_index=True)
        downsampled_coords = coords[unique_indices]

        # 4. Convert back to sensor_msgs/PointCloud2
        header = msg.header
        fields = [
            PointField(
                name='x', offset=0, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name='y', offset=4, datatype=PointField.FLOAT32, count=1
            ),
            PointField(
                name='z', offset=8, datatype=PointField.FLOAT32, count=1
            ),
        ]

        output_msg = point_cloud2.create_cloud(
            header, fields, downsampled_coords
        )

        # 5. Publish
        self.publisher_.publish(output_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()