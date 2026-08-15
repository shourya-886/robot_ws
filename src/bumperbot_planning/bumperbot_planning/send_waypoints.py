#!/usr/bin/env python3
"""
Sends a fixed list of waypoints to nav2's FollowWaypoints action server.
Run this after real_robot.launch.py is fully up and localized.
"""
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints

# Edit these to match your saved map's coordinates before running.
WAYPOINTS = [
    {"x": 1.0, "y": 0.0, "yaw": 0.0},
    {"x": 2.0, "y": 0.0, "yaw": 0.0},
]


def make_pose(node, wp):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = float(wp["x"])
    pose.pose.position.y = float(wp["y"])
    yaw = float(wp.get("yaw", 0.0))
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    rclpy.init()
    node = Node("send_waypoints")

    poses = [make_pose(node, wp) for wp in WAYPOINTS]
    client = ActionClient(node, FollowWaypoints, "follow_waypoints")

    node.get_logger().info("Waiting for follow_waypoints action server...")
    if not client.wait_for_server(timeout_sec=15.0):
        node.get_logger().error("follow_waypoints action server not available, aborting")
        rclpy.shutdown()
        sys.exit(1)

    goal_msg = FollowWaypoints.Goal()
    goal_msg.poses = poses

    node.get_logger().info(f"Sending {len(poses)} waypoints...")
    send_goal_future = client.send_goal_async(goal_msg)
    rclpy.spin_until_future_complete(node, send_goal_future)
    goal_handle = send_goal_future.result()

    if not goal_handle.accepted:
        node.get_logger().error("Waypoint goal was rejected")
        rclpy.shutdown()
        sys.exit(1)

    node.get_logger().info("Waypoint goal accepted, following...")
    result_future = goal_handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future)
    result = result_future.result().result

    if result.missed_waypoints:
        node.get_logger().warn(f"Missed waypoints: {list(result.missed_waypoints)}")
    else:
        node.get_logger().info("All waypoints completed successfully.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()