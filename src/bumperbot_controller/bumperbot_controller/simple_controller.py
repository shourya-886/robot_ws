#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.constants import S_TO_NS
from rclpy.time import Time
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math
from tf_transformations import quaternion_from_euler


class SimpleController(Node):

    def __init__(self):
        """
        Initializes the controller node, sets up kinematics parameters, 
        and prepares the Odometry and Transform Broadcaster for localization.
        """
        super().__init__("simple_controller") # Calls the constructor of the base Node class to name the node.
        self.declare_parameter("wheel_radius", 0.05) # Declares the robot's wheel radius parameter with a default value.
        self.declare_parameter("wheel_separation", 0.2286) # Declares the distance between the two wheels as a parameter.

        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value # Stores the wheel radius locally as a floating-point number.
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value # Stores the wheel separation locally as a floating-point number.

        self.get_logger().info("Using wheel radius %d" % self.wheel_radius_) # Logs the current wheel radius for user confirmation.
        self.get_logger().info("Using wheel separation %d" % self.wheel_separation_) # Logs the current wheel separation for user confirmation.

        self.left_wheel_prev_pos_ = 0.0 # Initializes the tracking of the left wheel's previous rotational position.
        self.right_wheel_prev_pos_ = 0.0 # Initializes the tracking of the right wheel's previous rotational position.
        self.x_ = 0.0 # Initializes the robot's global x-coordinate for odometry.
        self.y_ = 0.0 # Initializes the robot's global y-coordinate for odometry.
        self.theta_ = 0.0 # Initializes the robot's global heading (yaw) in radians.

        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray, "simple_velocity_controller/commands", 10) # Creates a publisher for motor velocity commands.
        self.vel_sub_ = self.create_subscription(TwistStamped, "bumperbot_controller/cmd_vel", self.velCallback, 10) # Subscribes to timestamped velocity commands from navigation.
        self.joint_sub_ = self.create_subscription(JointState,"joint_states", self.jointCallback, 10) # Subscribes to wheel encoder data via the joint_states topic.
        self.odom_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom", 10) # Creates a publisher for the robot's calculated odometry.

        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2], # Defines linear component of the kinematic conversion matrix.
                                           [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]]) # Defines angular component of the kinematic conversion matrix.
        self.get_logger().info("The conversion matrix is %s" % self.speed_conversion_) # Logs the resulting speed conversion matrix.

        # Fill the Odometry message with invariant parameters
        self.odom_msg_ = Odometry() # Initializes the Odometry message structure to be reused.
        self.odom_msg_.header.frame_id = "odom" # Sets the fixed coordinate frame to "odom".
        self.odom_msg_.child_frame_id = "base_footprint" # Sets the moving coordinate frame to the robot's base.
        self.odom_msg_.pose.pose.orientation.x = 0.0 # Sets initial orientation X to neutral.
        self.odom_msg_.pose.pose.orientation.y = 0.0 # Sets initial orientation Y to neutral.
        self.odom_msg_.pose.pose.orientation.z = 0.0 # Sets initial orientation Z to neutral.
        self.odom_msg_.pose.pose.orientation.w = 1.0 # Sets initial orientation W to 1.0 (identity quaternion).

        # Fill the TF message
        self.br_ = TransformBroadcaster(self) # Creates a broadcaster to send coordinate frame transformations.
        self.transform_stamped_ = TransformStamped() # Initializes the TF message structure for frequent updates.
        self.transform_stamped_.header.frame_id = "odom" # Sets the parent frame of the transform.
        self.transform_stamped_.child_frame_id = "base_footprint" # Sets the child frame of the transform.

        self.prev_time_ = self.get_clock().now() # Captures the start time to calculate the initial time delta.


    def velCallback(self, msg):
        """
        Converts commanded robot linear and angular velocities into specific 
        wheel speeds using the inverse differential kinematic model.
        """
        robot_speed = np.array([[msg.twist.linear.x], # Places the desired linear x speed into a vector.
                                [msg.twist.angular.z]]) # Places the desired angular z speed into the same vector.
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed) # Multiplies the inverted matrix by target speeds to get wheel velocities.

        wheel_speed_msg = Float64MultiArray() # Creates the array message for motor commands.
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]] # Packs the right and left wheel speeds into the message data.

        self.wheel_cmd_pub_.publish(wheel_speed_msg) # Publishes the calculated velocities to the hardware controller.

    
    def jointCallback(self, msg):
        """
        Calculates current robot velocity and updates global pose (odometry) 
        by integrating wheel encoder changes over time.
        """
        dp_left = msg.position[0] - self.left_wheel_prev_pos_ # Calculates the change in left wheel position.
        dp_right = msg.position[1] - self.right_wheel_prev_pos_ # Calculates the change in right wheel position.
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_ # Calculates the elapsed time since the previous update.

        # Actualize the prev pose for the next itheration
        self.left_wheel_prev_pos_ = msg.position[0] # Stores the current left wheel position for the next callback.
        self.right_wheel_prev_pos_ = msg.position[1] # Stores the current right wheel position for the next callback.
        self.prev_time_ = Time.from_msg(msg.header.stamp) # Stores the current timestamp for the next callback.

        # Calculate the rotational speed of each wheel
        fi_left = dp_left / (dt.nanoseconds / S_TO_NS) # Computes left wheel angular velocity in rad/s.
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS) # Computes right wheel angular velocity in rad/s.

        # Calculate the linear and angular velocity
        linear = (self.wheel_radius_ * fi_right + self.wheel_radius_ * fi_left) / 2 # Calculates the current robot linear velocity.
        angular = (self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left) / self.wheel_separation_ # Calculates the current robot angular velocity.

        # Calculate the position increment
        d_s = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left) / 2 # Computes the linear distance traveled during this step.
        d_theta = (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left) / self.wheel_separation_ # Computes the change in heading during this step.
        self.theta_ += d_theta # Integrates the change in heading into the global heading.
        self.x_ += d_s * math.cos(self.theta_) # Updates global X position using trigonometry and distance.
        self.y_ += d_s * math.sin(self.theta_) # Updates global Y position using trigonometry and distance.
        
        # Compose and publish the odom message
        q = quaternion_from_euler(0, 0, self.theta_) # Converts the Euler yaw angle into a quaternion for ROS.
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg() # Stamps the odometry message with the current time.
        self.odom_msg_.pose.pose.position.x = self.x_ # Sets the x position in the odometry message.
        self.odom_msg_.pose.pose.position.y = self.y_ # Sets the y position in the odometry message.
        self.odom_msg_.pose.pose.orientation.x = q[0] # Sets the quaternion X component.
        self.odom_msg_.pose.pose.orientation.y = q[1] # Sets the quaternion Y component.
        self.odom_msg_.pose.pose.orientation.z = q[2] # Sets the quaternion Z component.
        self.odom_msg_.pose.pose.orientation.w = q[3] # Sets the quaternion W component.
        self.odom_msg_.twist.twist.linear.x = linear # Adds the current linear velocity to the odometry message.
        self.odom_msg_.twist.twist.angular.z = angular # Adds the current angular velocity to the odometry message.
        self.odom_pub_.publish(self.odom_msg_) # Publishes the completed odometry message.

        # TF
        self.transform_stamped_.transform.translation.x = self.x_ # Sets the X translation for the TF frame.
        self.transform_stamped_.transform.translation.y = self.y_ # Sets the Y translation for the TF frame.
        self.transform_stamped_.transform.rotation.x = q[0] # Sets the rotation X component for the TF frame.
        self.transform_stamped_.transform.rotation.y = q[1] # Sets the rotation Y component for the TF frame.
        self.transform_stamped_.transform.rotation.z = q[2] # Sets the rotation Z component for the TF frame.
        self.transform_stamped_.transform.rotation.w = q[3] # Sets the rotation W component for the TF frame.
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg() # Updates the timestamp for the TF transform.
        self.br_.sendTransform(self.transform_stamped_) # Broadcasts the transform to the TF tree.


def main():
    """
    Standard main function to initialize ROS2, spin the SimpleController 
    node, and handle clean shutdown.
    """
    rclpy.init() # Initializes the ROS2 Python client library.

    simple_controller = SimpleController() # Instantiates the SimpleController node.
    rclpy.spin(simple_controller) # Keeps the node alive and responsive to incoming messages.
    
    simple_controller.destroy_node() # Cleans up resources allocated to the node.
    rclpy.shutdown() # Shuts down the ROS2 context.


if __name__ == '__main__':
    main() # Executes the main entry point.