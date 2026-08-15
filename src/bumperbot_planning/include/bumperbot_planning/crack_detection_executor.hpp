#ifndef CRACK_DETECTION_EXECUTOR_HPP
#define CRACK_DETECTION_EXECUTOR_HPP

#include <string>
#include <memory>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_core/waypoint_task_executor.hpp"
#include "nav2_msgs/action/spin.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace bumperbot_planning
{

// Stops at a waypoint, spins right for an inspection view, publishes a
// single camera frame for the crack-detection (YOLO) node to consume, then
// spins back left. This plugin never forwards a continuous image stream --
// it only ever publishes one message per waypoint, right after the spin.
class CrackDetectionExecutor : public nav2_core::WaypointTaskExecutor
{
public:
  using Spin = nav2_msgs::action::Spin;
  using GoalHandleSpin = rclcpp_action::ClientGoalHandle<Spin>;

  CrackDetectionExecutor() = default;
  ~CrackDetectionExecutor() override = default;

  void initialize(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    const std::string & plugin_name) override;

  bool processAtWaypoint(
    const geometry_msgs::msg::PoseStamped & curr_pose,
    const int & curr_waypoint_index) override;

private:
  // Blocks the calling thread until the /spin action finishes or times out.
  // Returns true on successful completion.
  bool doSpin(double target_yaw_rad, double timeout_sec);

  // Caches the latest frame from the camera topic. Does NOT republish --
  // this callback only updates latest_frame_, nothing more.
  void cameraCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  // Publishes exactly one copy of the currently cached frame to the
  // inspection image topic. Returns false if no frame has arrived yet.
  bool publishInspectionFrame(int waypoint_index);

  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node_;
  rclcpp::Logger logger_{rclcpp::get_logger("CrackDetectionExecutor")};
  std::string plugin_name_;

  rclcpp_action::Client<Spin>::SharedPtr spin_client_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr inspection_image_pub_;

  std::mutex frame_mutex_;
  sensor_msgs::msg::Image::SharedPtr latest_frame_;

  // Parameters
  double right_turn_rad_{-1.5708};   // -90 degrees, spin is signed (negative = clockwise/right)
  double left_turn_rad_{1.5708};     // +90 degrees, back to original heading
  double spin_timeout_sec_{15.0};
  std::string camera_topic_{"/camera/image_raw"};
  std::string inspection_image_topic_{"/waypoint/inspection_image"};
};

}  // namespace bumperbot_planning

#endif  // CRACK_DETECTION_EXECUTOR_HPP
