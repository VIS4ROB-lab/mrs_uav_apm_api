#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandHome


class AutoSetHome(Node):
    def __init__(self):
        super().__init__('auto_set_home')

        self.connected = False
        self.ekf_ok = False
        self.on_ground = False
        self.home_set = False

        self.ekf_ok_count = 0
        self.required_ekf_ok_count = 10  # ~0.5s at 20 Hz

        self.create_subscription(
            State,
            'mavros/state',
            self.state_cb,
            10
        )

        self.create_subscription(
            ExtendedState,
            'mavros/extended_state',
            self.extended_state_cb,
            10
        )

        self.home_client = self.create_client(
            CommandHome,
            'mavros/cmd/set_home'
        )

        self.get_logger().info('Waiting for mavros/cmd/set_home service...')
        self.home_client.wait_for_service()
        self.get_logger().info('AutoSetHome node started')

    def state_cb(self, msg: State):
        self.connected = msg.connected

    def extended_state_cb(self, msg: ExtendedState):
        self.ekf_ok = msg.ekf_ok
        self.on_ground = (
            msg.landed_state == ExtendedState.LANDED_STATE_ON_GROUND
        )

        if self.ekf_ok:
            self.ekf_ok_count += 1
        else:
            self.ekf_ok_count = 0

        if self.ready_to_set_home():
            self.set_home()

    def ready_to_set_home(self):
        return (
            self.connected and
            self.on_ground and
            not self.home_set and
            self.ekf_ok_count >= self.required_ekf_ok_count
        )

    def set_home(self):
        self.get_logger().info(
            'EKF stable + ON_GROUND → setting home position'
        )

        req = CommandHome.Request()
        req.current_gps = True
        req.latitude = 0.0
        req.longitude = 0.0
        req.altitude = 0.0

        future = self.home_client.call_async(req)
        future.add_done_callback(self.home_response_cb)

        # Prevent re-entry
        self.home_set = True

    def home_response_cb(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info('Home position set successfully')
            else:
                self.get_logger().warn('Set home failed')
                self.home_set = False
        except Exception as e:
            self.get_logger().error(f'Set home service call failed: {e}')
            self.home_set = False


def main():
    rclpy.init()
    node = AutoSetHome()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()