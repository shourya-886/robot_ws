import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import GetTransform
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster, TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler, quaternion_multiply, quaternion_inverse


class SimpleTfKinematics(Node):

    def __init__(self):
        super().__init__("simple_tf_kinematics")

        self.static_tf_broadcaster_ = StaticTransformBroadcaster(self)

        self.static_tf_broadcaster_.header.stamp = self.get_clock().now().to_msg()
        self.static_tf_broadcaster_.header.frame_id = "world"
        self.static_tf_broadcaster_.child_frame_id = "base_link"

def main():
    rclpy.init()
    simple_tf_kinematics = SimpleTfKinematics()
    rclpy.spin(simple_tf_kinematics)
    simple_tf_kinematics.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()