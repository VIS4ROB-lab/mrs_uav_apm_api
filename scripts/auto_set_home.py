#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandHome, SetMode, SetGPOrigin


class AutoSetHome(Node):
    def __init__(self):
        super().__init__('auto_set_home')

        # -------- Parameters --------
        self.stable_samples_required = 100  # 50 cycles ~2.5 s at 20 Hz
        self.max_delta_xy = 0.1           # meters
        self.max_delta_z = 0.1            # meters
        self.stabilize_mode = 'STABILIZE'
        self.post_home_mode = 'LOITER'

        # -------- State --------
        self.connected = False
        self.armed = False
        self.mode = ''
        self.pose_stable_count = 0
        self.last_pose = None

        self.ekf_origin_set = False
        self.waiting_for_stabilize = False
        self.home_set = False
        self.switching_to_loiter = False

        # -------- Subscribers --------
        self.create_subscription(State, 'mavros/state', self.state_cb, 1)
        self.create_subscription(PoseStamped, 'mavros/local_position/pose', self.pose_cb, 1)

        # -------- Services --------
        self.gp_origin_client = self.create_client(SetGPOrigin, 'mavros/global_position/set_gp_origin')
        self.set_home_client = self.create_client(CommandHome, 'mavros/cmd/set_home')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')

        self.get_logger().info('Waiting for MAVROS services...')
        self.gp_origin_client.wait_for_service()
        self.set_home_client.wait_for_service()
        self.set_mode_client.wait_for_service()
        self.get_logger().info('Auto-home node started (EKF origin + pose stability)')

    # ---------------- Callbacks ----------------

    def state_cb(self, msg: State):
        self.connected = msg.connected
        self.armed = msg.armed
        self.mode = msg.mode

        # If waiting for STABILIZE confirmation
        if self.waiting_for_stabilize and self.mode == self.stabilize_mode:
            self.get_logger().info('STABILIZE confirmed → setting home')
            self.waiting_for_stabilize = False
            self.call_set_home()

        # If waiting for LOITER confirmation
        if self.switching_to_loiter and self.mode == self.post_home_mode:
            self.get_logger().info(f'{self.post_home_mode} confirmed → auto-home complete')
            self.switching_to_loiter = False

    def pose_cb(self, msg: PoseStamped):
        if not self.connected or self.armed or self.home_set:
            return

        # Compute deltas for stability
        if self.last_pose is None:
            self.last_pose = msg.pose.position
            return

        dx = msg.pose.position.x - self.last_pose.x
        dy = msg.pose.position.y - self.last_pose.y
        dz = msg.pose.position.z - self.last_pose.z

        if abs(dx) <= self.max_delta_xy and abs(dy) <= self.max_delta_xy and abs(dz) <= self.max_delta_z:
            self.pose_stable_count += 1
        else:
            self.pose_stable_count = 0

        self.last_pose = msg.pose.position

        # Only trigger EKF origin + home after stable pose
        if self.pose_stable_count >= self.stable_samples_required:
            self.trigger_home_sequence()

    # ---------------- Logic ----------------

    def trigger_home_sequence(self):
        if not self.ekf_origin_set:
            self.set_ekf_origin()
        elif self.mode != self.stabilize_mode:
            self.get_logger().info(f'Pose stable → switching to {self.stabilize_mode}')
            self.waiting_for_stabilize = True
            self.call_set_mode(self.stabilize_mode)
        elif not self.home_set:
            self.call_set_home()

    # -------- Service calls --------

    def set_ekf_origin(self):
        self.get_logger().info('Setting EKF origin...')
        req = SetGPOrigin.Request()
        req.latitude = 0.0
        req.longitude = 0.0
        req.altitude = 0.0
        req.current_gps = False  # Using external nav (Vicon)
        future = self.gp_origin_client.call_async(req)
        future.add_done_callback(self.ekf_origin_response)

    def ekf_origin_response(self, future):
        try:
            resp = future.result()
            self.get_logger().info('EKF origin set successfully')
            self.ekf_origin_set = True
        except Exception as e:
            self.get_logger().error(f'Setting EKF origin failed: {e}')

    def call_set_mode(self, mode: str):
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)

    def call_set_home(self):
        self.get_logger().info('Calling SET_HOME...')
        req = CommandHome.Request()
        req.current_gps = True
        req.latitude = 0.0
        req.longitude = 0.0
        req.altitude = 0.0
        future = self.set_home_client.call_async(req)
        future.add_done_callback(self.home_response)

        self.home_set = True

    def home_response(self, future):
        try:
            resp = future.result()
            if resp.success:
                self.get_logger().info('Home set successfully → switching to LOITER')
                self.switching_to_loiter = True
                self.call_set_mode(self.post_home_mode)
            else:
                self.get_logger().warn(f'SET_HOME failed (result={resp.result}), retrying')
                # reset pose counter to retry
                self.pose_stable_count = 0
        except Exception as e:
            self.get_logger().error(f'SET_HOME service call failed: {e}')
            self.pose_stable_count = 0


def main():
    rclpy.init()
    node = AutoSetHome()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()