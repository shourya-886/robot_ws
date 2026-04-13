#include "bumperbot_controller/simple_controller.hpp"
#include <Eigen/Geometry>
#include <tf2/LinearMath/Quaternion.h>


using std::placeholders::_1;


SimpleController::SimpleController(const std::string& name)
                                  : Node(name)
                                  , left_wheel_prev_pos_(0.0)
                                  , right_wheel_prev_pos_(0.0)
                                  , x_(0.0)
                                  , y_(0.0)
                                  , theta_(0.0)
{
    /**
     * Initializes the C++ implementation of the controller node, setting up 
     * Eigen matrices for kinematics and initializing ROS 2 publishers and subscribers.
     */
    declare_parameter("wheel_radius", 0.033); // Registers the wheel radius parameter with a default value of 33mm.
    declare_parameter("wheel_separation", 0.17); // Registers the wheel separation parameter with a default value of 170mm.
    wheel_radius_ = get_parameter("wheel_radius").as_double(); // Fetches the wheel radius value from the parameter server.
    wheel_separation_ = get_parameter("wheel_separation").as_double(); // Fetches the wheel separation value from the parameter server.
    RCPP_INFO_STREAM(get_logger(), "Using wheel radius " << wheel_radius_); // Logs the active wheel radius to the console.
    RCPP_INFO_STREAM(get_logger(), "Using wheel separation " << wheel_separation_); // Logs the active wheel separation to the console.
    wheel_cmd_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/simple_velocity_controller/commands", 10); // Sets up the publisher for sending velocity commands to the hardware controller.
    vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>("/bumperbot_controller/cmd_vel", 10, std::bind(&SimpleController::velCallback, this, _1)); // Subscribes to input velocity commands with a bound callback function.
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&SimpleController::jointCallback, this, _1)); // Subscribes to wheel joint states for encoder-based odometry.
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/bumperbot_controller/odom", 10); // Sets up the publisher for the odometry message.

    speed_conversion_ << wheel_radius_/2, wheel_radius_/2, wheel_radius_/wheel_separation_, -wheel_radius_/wheel_separation_; // Fills the Eigen matrix with differential drive kinematic coefficients.
    RCPP_INFO_STREAM(get_logger(), "The conversion matrix is \n" << speed_conversion_); // Logs the matrix for verification.

    // Fill the Odometry message with invariant parameters
    odom_msg_.header.frame_id = "odom"; // Assigns the fixed frame ID to the odometry message.
    odom_msg_.child_frame_id = "base_footprint"; // Assigns the child frame ID (robot base) to the odometry message.
    odom_msg_.pose.pose.orientation.x = 0.0; // Sets initial orientation X to zero.
    odom_msg_.pose.pose.orientation.y = 0.0; // Sets initial orientation Y to zero.
    odom_msg_.pose.pose.orientation.z = 0.0; // Sets initial orientation Z to zero.
    odom_msg_.pose.pose.orientation.w = 1.0; // Sets initial orientation W to identity (no rotation).

    transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this); // Initializes the broadcaster for coordinate transformations.
    transform_stamped_.header.frame_id = "odom"; // Sets the parent frame ID for the TF transform.
    transform_stamped_.child_frame_id = "base_footprint"; // Sets the child frame ID for the TF transform.

    prev_time_ = get_clock()->now(); // Captures the initial clock time.
}


void SimpleController::velCallback(const geometry_msgs::msg::TwistStamped &msg)
{
    /**
     * Converts desired robot twist (linear and angular) into individual wheel
     * speeds using Eigen matrix inversion and publishes the results.
     */
    Eigen::Vector2d robot_speed(msg.twist.linear.x, msg.twist.angular.z); // Creates a vector for the robot's target linear and angular velocity.
    Eigen::Vector2d wheel_speed = speed_conversion_.inverse() * robot_speed; // Solves the inverse kinematics to find required wheel speeds.
    std_msgs::msg::Float64MultiArray wheel_speed_msg; // Initializes the array message container.
    wheel_speed_msg.data.push_back(wheel_speed.coeff(1)); // Adds the right wheel speed to the data array.
    wheel_speed_msg.data.push_back(wheel_speed.coeff(0)); // Adds the left wheel speed to the data array.
    
    wheel_cmd_pub_->publish(wheel_speed_msg); // Publishes the motor commands.
}


void SimpleController::jointCallback(const sensor_msgs::msg::JointState &state)
{
    /**
     * Performs forward kinematics to calculate current robot velocity and 
     * integrates the position to broadcast the odometry transform.
     */
    double dp_left = state.position.at(0) - left_wheel_prev_pos_; // Calculates change in left wheel position.
    double dp_right = state.position.at(1) - right_wheel_prev_pos_; // Calculates change in right wheel position.
    rclcpp::Time msg_time = state.header.stamp; // Extracts the timestamp from the joint state message.
    rclcpp::Duration dt = msg_time - prev_time_; // Calculates the time difference since the last update.

    // Actualize the prev pose for the next itheration
    left_wheel_prev_pos_ = state.position.at(0); // Updates previous left wheel position.
    right_wheel_prev_pos_ = state.position.at(1); // Updates previous right wheel position.
    prev_time_ = state.header.stamp; // Updates previous timestamp.

    // Calculate the rotational speed of each wheel
    double fi_left = dp_left / dt.seconds(); // Calculates left wheel angular velocity.
    double fi_right = dp_right / dt.seconds(); // Calculates right wheel angular velocity.

    // Calculate the linear and angular velocity
    double linear = (wheel_radius_ * fi_right + wheel_radius_ * fi_left) / 2; // Computes current linear velocity.
    double angular = (wheel_radius_ * fi_right - wheel_radius_ * fi_left) / wheel_separation_; // Computes current angular velocity.

    // Calculate the position increment
    double d_s = (wheel_radius_ * dp_right + wheel_radius_ * dp_left) / 2; // Computes distance increment.
    double d_theta = (wheel_radius_ * dp_right - wheel_radius_ * dp_left) / wheel_separation_; // Computes heading increment.
    theta_ += d_theta; // Updates total heading.
    x_ += d_s * cos(theta_); // Updates global X position.
    y_ += d_s * sin(theta_); // Updates global Y position.

    // Compose and publish the odom message
    tf2::Quaternion q; // Creates a quaternion object for orientation.
    q.setRPY(0, 0, theta_); // Converts Euler yaw to quaternion.
    odom_msg_.header.stamp = get_clock()->now(); // Stamps the message with current time.
    odom_msg_.pose.pose.position.x = x_; // Updates X in odom message.
    odom_msg_.pose.pose.position.y = y_; // Updates Y in odom message.
    odom_msg_.pose.pose.orientation.x = q.getX(); // Updates orientation X in odom message.
    odom_msg_.pose.pose.orientation.y = q.getY(); // Updates orientation Y in odom message.
    odom_msg_.pose.pose.orientation.z = q.getZ(); // Updates orientation Z in odom message.
    odom_msg_.pose.pose.orientation.w = q.getW(); // Updates orientation W in odom message.
    odom_msg_.twist.twist.linear.x = linear; // Updates linear velocity in odom message.
    odom_msg_.twist.twist.angular.z = angular; // Updates angular velocity in odom message.
    odom_pub_->publish(odom_msg_); // Publishes the odometry data.

    // TF
    transform_stamped_.transform.translation.x = x_; // Sets X translation for TF.
    transform_stamped_.transform.translation.y = y_; // Sets Y translation for TF.
    transform_stamped_.transform.rotation.x = q.getX(); // Sets orientation X for TF.
    transform_stamped_.transform.rotation.y = q.getY(); // Sets orientation Y for TF.
    transform_stamped_.transform.rotation.z = q.getZ(); // Sets orientation Z for TF.
    transform_stamped_.transform.rotation.w = q.getW(); // Sets orientation W for TF.
    transform_stamped_.header.stamp = get_clock()->now(); // Stamps the TF message.
    transform_broadcaster_->sendTransform(transform_stamped_); // Broadcasts the frame transform.
}


int main(int argc, char* argv[])
{
  /**
   * Initializes the ROS 2 environment, creates the SimpleController node, 
   * and enters the spin loop for callback execution.
   */
  rclcpp::init(argc, argv); // Initializes rclcpp.
  auto node = std::make_shared<SimpleController>("simple_controller"); // Instantiates the controller node.
  rclcpp::spin(node); // Loops to process incoming messages and callbacks.
  rclcpp::shutdown(); // Cleans up the ROS 2 context.
  return 0; // Exits the program.
}