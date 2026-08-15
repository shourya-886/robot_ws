#!/usr/bin/env python3
"""
Sends a fixed list of waypoints to nav2's FollowWaypoints action server.

Run this after real_robot.launch.py is fully up (map served/localized,
Nav2 lifecycle nodes active). It will block until the whole waypoint
sequence -- including the stop/spin-right/detect/spin-left choreography
at each point, handled by CrackDetectionExecutor -- completes.

Waypoint coordinates are frame "map" by default. Edit WAYPOINTS below,
or point --waypoints-file at a YAML file with the same structure, to
match your actual test site without rebuilding.
"""
import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
import yaml

# Fallback waypoints if no --waypoints-file is given. x, y in meters, map frame.
# yaw in radians. Edit these to match your test area, or use a YAML file instead.
WAYPOINTS = [
    {"x": 1.0, "y": 0.0, "yaw": 0.0},
    {"x": 2.0, "y": 0.0, "yaw": 0.0},
]


def load_waypoints(path):
    with open(path, "r") as f:
        data = yaml.safe_load(f)
    return data["waypoints"]


def make_pose(node, wp):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = float(wp["x"])
    pose.pose.position.y = float(wp["y"])
    yaw = float(wp.get("yaw", 0.0))
    # yaw -> quaternion (z, w only, since this is a planar robot)
    import math
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--waypoints-file", type=str, default=None,
                         help="YAML file with a top-level 'waypoints' list of {x, y, yaw}")
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = Node("send_waypoints")

    waypoints = load_waypoints(args.waypoints_file) if args.waypoints_file else WAYPOINTS
    poses = [make_pose(node, wp) for wp in waypoints]

    client = ActionClient(node, FollowWaypoints, "follow_waypoints")

    node.get_logger().info(f"Waiting for follow_waypoints action server...")
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
