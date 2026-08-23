#!/usr/bin/env python3

"""
WPS -- Waypoint Sender.

Sends a fixed list of waypoints to Nav2's FollowWaypoints action server.

The node waits for both:
    1. navigate_to_pose
    2. follow_waypoints

before sending the waypoint goal.
"""

import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints, NavigateToPose


#keep waypoint 1's y and waypoint's y the same
#y1=y2
#x2=x3
#x2-x1 = -1.05865934491
# Hardcoded waypoints, obtained via /clicked_point in RViz2.
WAYPOINTS = [
    {
        "x": 0.5537700688838959,
        "y": 0.034409839659929276,
        "yaw": -1.3962634  # Adjust orientation as needed current is 80deg
    },
    {
        "x": 1.5024294137954712,
        "y": 0.034409839659929276,
        "yaw": -1.57079632679  # Adjust orientation as needed
    },
    {
        "x": 1.5024294137954712,
        "y": -0.9565174913406372,
        "yaw": -2.967059728392  # Adjust orientation as needed current is 170deg
    },
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

    node = Node("waypoint_sender")

    # Create the NavigateToPose action client.
    navigate_client = ActionClient(
        node,
        NavigateToPose,
        "navigate_to_pose"
    )

    # Create the FollowWaypoints action client.
    follow_waypoints_client = ActionClient(
        node,
        FollowWaypoints,
        "follow_waypoints"
    )

    # ---------------------------------------------------------
    # Wait for Nav2's navigate_to_pose action server.
    # ---------------------------------------------------------

    node.get_logger().info(
        "Waiting for navigate_to_pose action server..."
    )
    

    if not navigate_client.wait_for_server(timeout_sec=15.0):
        node.get_logger().error(
            "navigate_to_pose action server not available, aborting"
        )

        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    node.get_logger().info(
        "navigate_to_pose action server is ready."
    )
    time.sleep(9)

    # ---------------------------------------------------------
    # Wait for the FollowWaypoints action server.
    # ---------------------------------------------------------

    node.get_logger().info(
        "Waiting for follow_waypoints action server..."
    )


    if not follow_waypoints_client.wait_for_server(timeout_sec=15.0):
        node.get_logger().error(
            "follow_waypoints action server not available, aborting"
        )

        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    node.get_logger().info(
        "follow_waypoints action server is ready."
    )

    # ---------------------------------------------------------
    # Prepare waypoint poses.
    # ---------------------------------------------------------

    poses = [
        make_pose(node, wp)
        for wp in WAYPOINTS
    ]

    # ---------------------------------------------------------
    # Send waypoint goal.
    # ---------------------------------------------------------

    goal_msg = FollowWaypoints.Goal()
    goal_msg.poses = poses

    node.get_logger().info(
        f"Sending {len(poses)} waypoints..."
    )

    send_goal_future = (
        follow_waypoints_client.send_goal_async(goal_msg)
    )

    rclpy.spin_until_future_complete(
        node,
        send_goal_future
    )

    goal_handle = send_goal_future.result()

    if goal_handle is None:
        node.get_logger().error(
            "Failed to receive waypoint goal response"
        )

        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    if not goal_handle.accepted:
        node.get_logger().error(
            "Waypoint goal was rejected"
        )

        node.destroy_node()
        rclpy.shutdown()
        sys.exit(1)

    node.get_logger().info(
        "Waypoint goal accepted, following..."
    )

    # ---------------------------------------------------------
    # Wait for the waypoint result.
    # ---------------------------------------------------------

    result_future = goal_handle.get_result_async()

    rclpy.spin_until_future_complete(
        node,
        result_future
    )

    result = result_future.result().result

    # ---------------------------------------------------------
    # Check waypoint result.
    # ---------------------------------------------------------

    if result.missed_waypoints:
        node.get_logger().warn(
            f"Missed waypoints: {list(result.missed_waypoints)}"
        )
    else:
        node.get_logger().info(
            "All waypoints completed successfully."
        )

    # ---------------------------------------------------------
    # Shutdown.
    # ---------------------------------------------------------

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()