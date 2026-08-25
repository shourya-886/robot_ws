import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_slam = LaunchConfiguration("use_slam")
    use_yolo = LaunchConfiguration("use_yolo")
    use_waypoint = LaunchConfiguration("use_waypoint")

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    use_yolo_arg = DeclareLaunchArgument(
        "use_yolo",
        default_value="false"
    )

    use_waypoint_arg = DeclareLaunchArgument(
        "use_waypoint",
        default_value="false"
    )

    yolo_model = LaunchConfiguration("yolo_model")
    yolo_model_arg = DeclareLaunchArgument(
        "yolo_model",
        default_value="/home/shourya/robot_ws/src/bumperbot_yolo/models/part12_crack_wromodel60.onnx"
    )


    hardware_interface = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_firmware"),
            "launch",
            "hardware_interface.launch.py"
        ),
    )

    laser_driver = Node(
            package="rplidar_ros",
            executable="rplidar_node",
            name="rplidar_node",
            parameters=[os.path.join(
                get_package_share_directory("bumperbot_bringup"),
                "config",
                "rplidar_a1.yaml"
            )],
            output="screen"
    )
    
    controller = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "controller.launch.py"
        ),
        launch_arguments={
            "use_simple_controller": "False",
            "use_python": "False"
        }.items(),
    )
    
    joystick = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_controller"),
            "launch",
            "joystick_teleop.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items()
    )

    imu_driver_node = Node(
        package="bumperbot_firmware",
        executable="mpu6050_driver.py"
    )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_localization"),
            "launch",
            "global_localization.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items(),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items(),
        condition=IfCondition(use_slam)
    )

    navigation = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_navigation"),
            "launch",
            "navigation.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items(),
    )

    camera_node = Node(
            package="bumperbot_yolo",
            executable="camera_node",
            name="camera_node",
            output="screen",
            condition=IfCondition(use_yolo)
        )

    yolo_node = Node(
            package="bumperbot_yolo",
            executable="main_ros",
            name="yolo_node",
            output="screen",
            parameters=[{"model": yolo_model}],
            condition=IfCondition(use_yolo)
        )

    waypoint_follower = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("bumperbot_navigation"),
            "launch",
            "waypoint.launch.py"
        ),
        launch_arguments={
            "use_sim_time": "False"
        }.items(),
        condition=IfCondition(use_waypoint)
    )

    # safety_stop = Node(
    #     package="bumperbot_utils",
    #     executable="safety_stop",
    #     output="screen",
    # )
    
    return LaunchDescription([
        use_slam_arg,
        use_yolo_arg,
        use_waypoint_arg,
        yolo_model_arg,
        hardware_interface,
        laser_driver,
        controller,
        joystick,
        imu_driver_node,
        localization,
        slam,
        navigation,
        camera_node,
        yolo_node,
        waypoint_follower,
    ])