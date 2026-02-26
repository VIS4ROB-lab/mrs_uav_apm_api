#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPointStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State
from mavros_msgs.msg import HomePosition
from mavros_msgs.srv import SetMode

class AutoSetHome(Node):
    def __init__(self):
        super().__init__('auto_set_home')

        # -------- Parameters --------
        self.stable_samples_required = 100  # ~2.5 s at 20 Hz
        self.max_delta_xy = 0.1
        self.max_delta_z = 0.1
        self.stabilize_mode = 'STABILIZE'
        self.post_home_mode = 'LOITER'

        # -------- State --------
        self.connected = False
        self.armed = False
        self.mode = ''
        self.pose_stable_count = 0
        self.last_pose = None
        self.last_orientation = None

        self.ekf_origin_set = False
        self.waiting_for_stabilize = False
        self.home_set = False
        self.switching_to_loiter = False
        self.gps_fix_received = False
        self.gps_latitude = 0.0
        self.gps_longitude = 0.0
        self.gps_altitude = 0.0

        # -------- Subscribers --------
        self.create_subscription(State, 'mavros/state', self.state_cb, 1)
        self.create_subscription(PoseStamped, 'mavros/local_position/pose', self.pose_cb, 1)
        self.create_subscription(NavSatFix, 'mavros/global_position/global', self.gps_cb, 1)
        self.create_subscription(GeoPointStamped, 'mavros/global_position/gp_origin', self.gp_origin_cb, 1)
        self.create_subscription(HomePosition, 'mavros/home_position/home', self.home_position_cb, 1)
    
        # -------- Publishers / Services --------
        self.ekf_origin_pub = self.create_publisher(GeoPointStamped, 'mavros/global_position/set_gp_origin', 1)
        self.home_position_pub = self.create_publisher(HomePosition, 'mavros/home_position/set', 1)
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')

        self.get_logger().info('Waiting for MAVROS services...')
        self.set_mode_client.wait_for_service()
        self.get_logger().info('Auto-set-home node started')

    # ---------------- Callbacks ----------------
    def home_position_cb(self, msg: HomePosition):
            if not self.home_set:
                self.get_logger().info('Home position confirmed from mavros/home_position/home')
                self.get_logger().info('Switching to LOITER')
                self.switching_to_loiter = True
                self.call_set_mode(self.post_home_mode)
                self.home_set = True
    def gp_origin_cb(self, msg: GeoPointStamped):
        if not self.ekf_origin_set:
            self.ekf_origin_set = True
            self.get_logger().info('EKF origin confirmed from mavros/global_position/gp_origin')

    def state_cb(self, msg: State):
        self.connected = msg.connected
        self.armed = msg.armed
        self.mode = msg.mode

        # STABILIZE confirmation
        if self.waiting_for_stabilize and self.mode == self.stabilize_mode:
            self.get_logger().info('STABILIZE confirmed → setting home')
            self.waiting_for_stabilize = False
            if self.last_pose is not None and self.last_orientation is not None:
                self.publish_home_position()

        # LOITER confirmation
        if self.switching_to_loiter and self.mode == self.post_home_mode:
            self.get_logger().info(f'{self.post_home_mode} confirmed → auto-home complete')
            self.switching_to_loiter = False

    def pose_cb(self, msg: PoseStamped):
        if not self.connected or self.armed or self.home_set:
            return

        # Compute deltas for stability
        if self.last_pose is None:
            self.last_pose = msg.pose.position
            self.last_orientation = msg.pose.orientation
            return

        dx = msg.pose.position.x - self.last_pose.x
        dy = msg.pose.position.y - self.last_pose.y
        dz = msg.pose.position.z - self.last_pose.z

        if abs(dx) <= self.max_delta_xy and abs(dy) <= self.max_delta_xy and abs(dz) <= self.max_delta_z:
            self.pose_stable_count += 1
        else:
            self.pose_stable_count = 0

        self.last_pose = msg.pose.position
        self.last_orientation = msg.pose.orientation

        if self.pose_stable_count >= self.stable_samples_required:
            self.trigger_home_sequence()

    def gps_cb(self, msg: NavSatFix):
        self.gps_latitude = msg.latitude
        self.gps_longitude = msg.longitude
        self.gps_altitude = msg.altitude
        if not self.gps_fix_received:
            self.gps_fix_received = True
            self.get_logger().info(
                f'GPS fix received: lat={self.gps_latitude:.8f}, '
                f'lon={self.gps_longitude:.8f}, alt={self.gps_altitude:.3f}'
            )

    # ---------------- Logic ----------------
    def trigger_home_sequence(self):
        if not self.gps_fix_received or self.last_pose is None or self.last_orientation is None:
            return

        if not self.ekf_origin_set:
            self.set_ekf_origin()
            time.sleep(2.0)  # Give EKF time to process origin
        elif self.mode != self.stabilize_mode:
            self.get_logger().info(f'Pose stable → switching to {self.stabilize_mode}')
            self.waiting_for_stabilize = True
            self.call_set_mode(self.stabilize_mode)
        elif not self.home_set and self.ekf_origin_set:
            self.publish_home_position()
            time.sleep(2.0)  # Give mavros time to process home position

    # -------- EKF origin --------
    def set_ekf_origin(self):
        self.get_logger().info('Publishing EKF origin (GeoPointStamped)...')
        msg = GeoPointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position.latitude = self.gps_latitude
        msg.position.longitude = self.gps_longitude
        msg.position.altitude = self.gps_altitude
        self.ekf_origin_pub.publish(msg)
        self.get_logger().info(
            f'EKF origin published successfully: lat={self.gps_latitude:.8f}, '
            f'lon={self.gps_longitude:.8f}, alt={self.gps_altitude:.3f}'
        )

    # -------- Service calls --------
    def call_set_mode(self, mode: str):
        req = SetMode.Request()
        req.custom_mode = mode
        self.set_mode_client.call_async(req)

    def publish_home_position(self):
        self.get_logger().info('Publishing HomePosition to mavros/home_position/set...')
        msg = HomePosition()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.geo.latitude = self.gps_latitude
        msg.geo.longitude = self.gps_longitude
        msg.geo.altitude = self.gps_altitude
        msg.position.x = self.last_pose.x
        msg.position.y = self.last_pose.y
        msg.position.z = self.last_pose.z
        msg.orientation.x = self.last_orientation.x
        msg.orientation.y = self.last_orientation.y
        msg.orientation.z = self.last_orientation.z
        msg.orientation.w = self.last_orientation.w
        msg.approach.x = 0.0
        msg.approach.y = 0.0
        msg.approach.z = 1.0
        self.home_position_pub.publish(msg)
        self.get_logger().info(
            f'HomePosition published: lat={self.gps_latitude:.8f}, '
            f'lon={self.gps_longitude:.8f}, alt={self.gps_altitude:.3f}'
        )

def main():
    rclpy.init()
    node = AutoSetHome()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.home_set:
                node.get_logger().info('Home position set, exiting node.')
                break
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()