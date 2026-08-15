#include "bumperbot_planning/crack_detection_executor.hpp"

#include <chrono>
#include <future>

using namespace std::chrono_literals;

namespace bumperbot_planning
{

void CrackDetectionExecutor::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name)
{
  parent_node_ = parent;
  plugin_name_ = plugin_name;
  auto node = parent_node_.lock();
  if (!node) {
    RCLCPP_ERROR(logger_, "Failed to lock parent node during initialize()");
    return;
  }
  logger_ = node->get_logger();

  // Declare parameters under the plugin's own namespace, e.g.
  // waypoint_follower.crack_detection_executor.right_turn_rad
  node->declare_parameter(plugin_name_ + ".right_turn_rad", right_turn_rad_);
  node->declare_parameter(plugin_name_ + ".left_turn_rad", left_turn_rad_);
  node->declare_parameter(plugin_name_ + ".spin_timeout_sec", spin_timeout_sec_);
  node->declare_parameter(plugin_name_ + ".camera_topic", camera_topic_);
  node->declare_parameter(plugin_name_ + ".inspection_image_topic", inspection_image_topic_);

  right_turn_rad_ = node->get_parameter(plugin_name_ + ".right_turn_rad").as_double();
  left_turn_rad_ = node->get_parameter(plugin_name_ + ".left_turn_rad").as_double();
  spin_timeout_sec_ = node->get_parameter(plugin_name_ + ".spin_timeout_sec").as_double();
  camera_topic_ = node->get_parameter(plugin_name_ + ".camera_topic").as_string();
  inspection_image_topic_ = node->get_parameter(plugin_name_ + ".inspection_image_topic").as_string();

  spin_client_ = rclcpp_action::create_client<Spin>(node, "spin");

  // Subscribed continuously, but this ONLY updates latest_frame_ -- it is
  // never republished as a stream. The only outgoing publish happens once
  // per waypoint, inside publishInspectionFrame().
  camera_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
    camera_topic_, rclcpp::SensorDataQoS(),
    std::bind(&CrackDetectionExecutor::cameraCallback, this, std::placeholders::_1));

  inspection_image_pub_ = node->create_publisher<sensor_msgs::msg::Image>(
    inspection_image_topic_, rclcpp::QoS(1));

  RCLCPP_INFO(
    logger_,
    "CrackDetectionExecutor initialized (right_turn=%.3f rad, left_turn=%.3f rad, "
    "camera_topic='%s', inspection_image_topic='%s')",
    right_turn_rad_, left_turn_rad_, camera_topic_.c_str(), inspection_image_topic_.c_str());
}

void CrackDetectionExecutor::cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(frame_mutex_);
  latest_frame_ = msg;
}

bool CrackDetectionExecutor::publishInspectionFrame(int waypoint_index)
{
  sensor_msgs::msg::Image::SharedPtr frame_to_publish;
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    frame_to_publish = latest_frame_;
  }

  if (!frame_to_publish) {
    RCLCPP_ERROR(
      logger_,
      "Waypoint %d: no camera frame received yet on '%s', cannot publish inspection image",
      waypoint_index, camera_topic_.c_str());
    return false;
  }

  inspection_image_pub_->publish(*frame_to_publish);
  RCLCPP_INFO(
    logger_, "Waypoint %d: published one frame to '%s' for crack detection",
    waypoint_index, inspection_image_topic_.c_str());
  return true;
}

bool CrackDetectionExecutor::doSpin(double target_yaw_rad, double timeout_sec)
{
  auto node = parent_node_.lock();
  if (!node) {
    RCLCPP_ERROR(logger_, "Parent node no longer valid, cannot spin");
    return false;
  }

  if (!spin_client_->wait_for_action_server(2s)) {
    RCLCPP_ERROR(logger_, "Spin action server not available");
    return false;
  }

  auto goal_msg = Spin::Goal();
  goal_msg.target_yaw = target_yaw_rad;

  auto goal_future = spin_client_->async_send_goal(goal_msg);

  // Block this thread (processAtWaypoint runs on the waypoint_follower's
  // action server thread, not the main executor, so blocking here is safe
  // and matches the synchronous nature of this task).
  if (goal_future.wait_for(std::chrono::duration<double>(timeout_sec)) !=
    std::future_status::ready)
  {
    RCLCPP_ERROR(logger_, "Timed out waiting for spin goal to be accepted");
    return false;
  }

  auto goal_handle = goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(logger_, "Spin goal was rejected by the behavior server");
    return false;
  }

  auto result_future = spin_client_->async_get_result(goal_handle);
  if (result_future.wait_for(std::chrono::duration<double>(timeout_sec)) !=
    std::future_status::ready)
  {
    RCLCPP_ERROR(logger_, "Timed out waiting for spin to complete");
    return false;
  }

  auto result = result_future.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    RCLCPP_ERROR(logger_, "Spin did not succeed (result code %d)", static_cast<int>(result.code));
    return false;
  }

  return true;
}

bool CrackDetectionExecutor::processAtWaypoint(
  const geometry_msgs::msg::PoseStamped & curr_pose,
  const int & curr_waypoint_index)
{
  (void)curr_pose;
  RCLCPP_INFO(logger_, "Arrived at waypoint %d -- starting inspection sequence", curr_waypoint_index);

  // Robot is already stopped by waypoint_follower at this point (it only
  // calls processAtWaypoint once the goal_checker reports arrival).

  RCLCPP_INFO(logger_, "Waypoint %d: turning right for inspection view", curr_waypoint_index);
  if (!doSpin(right_turn_rad_, spin_timeout_sec_)) {
    RCLCPP_ERROR(logger_, "Waypoint %d: right turn failed, aborting task at this waypoint", curr_waypoint_index);
    return false;
  }

  RCLCPP_INFO(logger_, "Waypoint %d: stopped, publishing inspection frame", curr_waypoint_index);
  bool publish_ok = publishInspectionFrame(curr_waypoint_index);
  if (!publish_ok) {
    RCLCPP_WARN(
      logger_,
      "Waypoint %d: failed to publish inspection frame, still returning to heading before moving on",
      curr_waypoint_index);
  }

  RCLCPP_INFO(logger_, "Waypoint %d: turning left to resume heading", curr_waypoint_index);
  if (!doSpin(left_turn_rad_, spin_timeout_sec_)) {
    RCLCPP_ERROR(logger_, "Waypoint %d: left turn failed", curr_waypoint_index);
    return false;
  }

  RCLCPP_INFO(logger_, "Waypoint %d: inspection sequence complete", curr_waypoint_index);
  return publish_ok;
}

}  // namespace bumperbot_planning

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(bumperbot_planning::CrackDetectionExecutor, nav2_core::WaypointTaskExecutor)
