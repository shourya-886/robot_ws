#ifndef SIMPLE_CONTROLLER_HPP // Header guard to prevent multiple inclusions of this header file.
#define SIMPLE_CONTROLLER_HPP // Defines the macro to signify the header is already included.

#include <rclcpp/rclcpp.hpp> // Includes the core ROS 2 C++ client library functionalities.
#include <geometry_msgs/msg/twist_stamped.hpp> // Includes message type for timestamped linear and angular velocities.
#include <Eigen/Core> // Includes the Eigen library for fixed-size matrices and basic linear algebra.
#include <std_msgs/msg/float64_multi_array.hpp> // Includes message type for sending arrays of floating-point numbers.
#include <sensor_msgs/msg/joint_state.hpp> // Includes message type for reporting robot joint positions and velocities.
#include <nav_msgs/msg/odometry.hpp> // Includes message type for estimated position and velocity in free space.
#include <geometry_msgs/msg/transform_stamped.hpp> // Includes message type for timestamped coordinate frame transformations.
#include <tf2_ros/transform_broadcaster.h> // Includes the utility for sending coordinate transforms to the TF system.


class SimpleController : public rclcpp::Node // Defines the SimpleController class inheriting from the ROS 2 Node class.
{
public:
    /**
     * Constructor for the SimpleController class that initializes the ROS 2 node 
     * with a specific name and sets up internal state variables.
     */
    SimpleController(const std::string& name); // Public constructor declaration taking the node name as an argument.

private:
    /**
     * Callback function for processing incoming velocity commands, converting 
     * robot twist into specific motor speed commands.
     */
    void velCallback(const geometry_msgs::msg::TwistStamped &msg); // Private method to handle incoming velocity (twist) messages.

    /**
     * Callback function for processing joint state updates, calculating the 
     * robot's current position and orientation via wheel odometry.
     */
    void jointCallback(const sensor_msgs::msg::JointState &msg); // Private method to handle incoming encoder data (joint states).
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr vel_sub_; // Shared pointer for the subscriber to the command velocity topic.
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_cmd_pub_; // Shared pointer for the publisher sending wheel speed commands.
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_; // Shared pointer for the subscriber to the joint states topic.
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_; // Shared pointer for the publisher broadcasting odometry data.

    // Odometry
    double wheel_radius_; // Variable to store the physical radius of the robot's wheels.
    double wheel_separation_; // Variable to store the distance between the left and right wheels.
    Eigen::Matrix2d speed_conversion_; // 2x2 Eigen matrix used to store the kinematic conversion coefficients.
    double right_wheel_prev_pos_; // Variable to track the previous rotation position of the right wheel.
    double left_wheel_prev_pos_; // Variable to track the previous rotation position of the left wheel.
    rclcpp::Time prev_time_; // Variable to store the timestamp of the last received joint state.
    nav_msgs::msg::Odometry odom_msg_; // Persistent odometry message object used to store and publish state.
    double x_; // Variable representing the robot's global X-coordinate position.
    double y_; // Variable representing the robot's global Y-coordinate position.
    double theta_; // Variable representing the robot's global orientation (heading) in radians.

    // TF
    std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_; // Unique pointer managing the life of the transform broadcaster object.
    geometry_msgs::msg::TransformStamped transform_stamped_; // Persistent transform message used to update coordinate frames in the TF tree.
};

#endif // SIMPLE_CONTROLLER_HPP // End of the header guard.