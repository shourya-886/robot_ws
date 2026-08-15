#!/usr/bin/env python3
"""
Stub crack-detection node.

Stands in for the real YOLOv11 pipeline while the waypoint-following
choreography (stop -> spin right -> stop -> detect -> spin left -> continue)
is being tested end-to-end without the actual vision stack running.

Exposes the same service interface the CrackDetectionExecutor plugin calls,
so swapping this out for the real YOLO node later is a drop-in replacement:
just point service_name at the real node and delete this file.
"""
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class CrackDetectionStub(Node):

    def __init__(self):
        super().__init__('crack_detection_stub')
        self.declare_parameter('detect_duration_sec', 2.0)
        self.declare_parameter('service_name', 'detect_crack')

        duration = self.get_parameter('detect_duration_sec').value
        service_name = self.get_parameter('service_name').value

        self._duration = duration
        self._srv = self.create_service(Trigger, service_name, self.handle_detect)
        self._call_count = 0

        self.get_logger().info(
            f"Crack detection stub ready on service '{service_name}' "
            f"(simulated inference time: {duration}s)"
        )

    def handle_detect(self, request, response):
        self._call_count += 1
        self.get_logger().info(f"[stub] Detection triggered (call #{self._call_count}) -- running inference...")

        time.sleep(self._duration)

        # Fake alternating result so terminal output is easy to eyeball during testing
        crack_found = (self._call_count % 2 == 1)
        if crack_found:
            self.get_logger().warn(
                f"[stub] CRACK DETECTED at waypoint (call #{self._call_count}) "
                f"-- confidence 0.87 (fake)"
            )
        else:
            self.get_logger().info(f"[stub] No crack detected (call #{self._call_count})")

        response.success = True
        response.message = "crack_detected" if crack_found else "no_crack"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = CrackDetectionStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
