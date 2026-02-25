#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandHome


class AutoSetHomePoseStability(Node):
    def __init__(self):
        super().__init__('auto_set_home_pose_stability')

        # --- Parameters ---
        self.required_stable_samples = 10
        self.max_delta_xy = 0.1  # meters
        self.max_delta_z  = 0.1  # meters

        # --- State ---
        self.connected = False
        self.armed = False
        self.home_set = False

        self.last_pose = None
        self.stable_count = 0

        # --- Subscribers ---
        self.create_subscription(
            State,
            'mavros/state',
            self.state_cb,
            1
        )

        self.create_subscription(
            PoseStamped,
            'mavros/local_position/pose',
            self.pose_cb,
            1
        )

        # --- Service ---
        self.home_client = self.create_client(
            CommandHome,
            'mavros/cmd/set_home'
        )

        self.get_logger().info('Waiting for mavros/cmd/set_home service...')
        self.home_client.wait_for_service()
        self.get_logger().info('Auto-set-home (pose stability) node started')

    # ---------------- Callbacks ----------------

    def state_cb(self, msg: State):
        self.connected = msg.connected
        self.armed = msg.armed

    def pose_cb(self, msg: PoseStamped):
        if self.home_set or not self.connected or self.armed:
            return

        if self.last_pose is None:
            self.last_pose = msg.pose.position
            return

        dx = msg.pose.position.x - self.last_pose.x
        dy = msg.pose.position.y - self.last_pose.y
        dz = msg.pose.position.z - self.last_pose.z

        if (
            abs(dx) <= self.max_delta_xy and
            abs(dy) <= self.max_delta_xy and
            abs(dz) <= self.max_delta_z
        ):
            self.stable_count += 1
        else:
            self.stable_count = 0

        self.last_pose = msg.pose.position

        if self.stable_count >= self.required_stable_samples:
            self.set_home()

    # ---------------- Home ----------------

    def set_home(self):
        self.get_logger().info(
            f'Pose stable for {self.stable_count} samples → setting home'
        )

        req = CommandHome.Request()
        req.current_gps = True
        req.latitude = 0.0
        req.longitude = 0.0
        req.altitude = 0.0

        future = self.home_client.call_async(req)
        future.add_done_callback(self.home_response_cb)

        # latch to prevent re-entry
        self.home_set = True

    def home_response_cb(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info('Home position set successfully')
            else:
                self.get_logger().warn('Set home failed — will retry')
                self.reset()
        except Exception as e:
            self.get_logger().error(f'Set home call failed: {e}')
            self.reset()

    def reset(self):
        self.home_set = False
        self.stable_count = 0
        self.last_pose = None


def main():
    rclpy.init()
    node = AutoSetHomePoseStability()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()