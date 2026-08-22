import os
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():

    # Turns raw /imu/out (gyro + accel, no orientation) into /imu/data
    # (gyro + accel + orientation quaternion) using a gyro/accel-only
    # (no magnetometer) Madgwick AHRS filter.
    madgwick_filter = Node(
        package="imu_filter_madgwick",
        executable="imu_filter_madgwick_node",
        name="imu_filter_madgwick_node",
        output="screen",
        parameters=[{
            "use_mag": False,
            "publish_tf": False,
            "world_frame": "enu",
            "fixed_frame": "odom",
            "gain": 0.1,
            "zeta": 0.0,
        }],
        remappings=[
            ("imu/data_raw", "/imu/out"),
            ("imu/data", "/imu/data"),
        ],
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "config",
            "ekf_real.yaml"
        )],
    )

    return LaunchDescription([
        madgwick_filter,
        ekf_node,
    ])