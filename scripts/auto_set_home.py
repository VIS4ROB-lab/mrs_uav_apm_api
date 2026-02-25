#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandHome, SetMode


class AutoSetHomeForceStabilize(Node):
    def __init__(self):
        super().__init__('auto_set_home_force_stabilize')

        # -------- Parameters --------
        self.required_stable_samples = 100   # ~1–1.5 s
        self.max_delta_xy = 0.1            # meters
        self.max_delta_z  = 0.1            # meters
        self.required_mode = 'STABILIZE'

        # -------- State --------
        self.connected = False
        self.armed = False
        self.mode = ''
        self.home_set = False

        self.last_pose = None
        self.stable_count = 0
        self.waiting_for_mode = False

        # -------- Subscribers --------
        self.create_subscription(State, 'mavros/state', self.state_cb, 1)
        self.create_subscription(
            PoseStamped,
            'mavros/local_position/pose',
            self.pose_cb,
            10
        )

        # -------- Services --------
        self.set_home_client = self.create_client(
            CommandHome, 'mavros/cmd/set_home'
        )
        self.set_mode_client = self.create_client(
            SetMode, 'mavros/set_mode'
        )

        self.get_logger().info('Waiting for MAVROS services...')
        self.set_home_client.wait_for_service()
        self.set_mode_client.wait_for_service()
        self.get_logger().info('Auto-set-home (force STABILIZE) node started')

    # ---------------- Callbacks ----------------

    def state_cb(self, msg: State):
        self.connected = msg.connected
        self.armed = msg.armed
        self.mode = msg.mode

        # If we requested STABILIZE and it is now active, set home
        if self.waiting_for_mode and self.mode == self.required_mode:
            self.get_logger().info('STABILIZE confirmed → setting home')
            self.waiting_for_mode = False
            self.call_set_home()

    def pose_cb(self, msg: PoseStamped):
        if (
            self.home_set or
            not self.connected or
            self.armed
        ):
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
            self.trigger_home_sequence()

    # ---------------- Logic ----------------

    def trigger_home_sequence(self):
        if self.mode != self.required_mode:
            self.get_logger().info(
                f'Pose stable → switching to {self.required_mode}'
            )
            self.waiting_for_mode = True
            self.call_set_mode(self.required_mode)
        else:
            self.call_set_home()

    def call_set_mode(self, mode: str):
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)

    def call_set_home(self):
        self.get_logger().info('Calling SET_HOME')

        req = CommandHome.Request()
        req.current_gps = True
        req.latitude = 0.0
        req.longitude = 0.0
        req.altitude = 0.0

        future = self.set_home_client.call_async(req)
        future.add_done_callback(self.home_response_cb)

        # Latch to prevent re-entry
        self.home_set = True

    def home_response_cb(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info('Home position set successfully')
            else:
                self.get_logger().warn(
                    f'SET_HOME failed (result={resp.result}), will retry'
                )
                self.reset()
        except Exception as e:
            self.get_logger().error(f'SET_HOME service call failed: {e}')
            self.reset()

    def reset(self):
        self.home_set = False
        self.waiting_for_mode = False
        self.stable_count = 0
        self.last_pose = None


def main():
    rclpy.init()
    node = AutoSetHomeForceStabilize()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()