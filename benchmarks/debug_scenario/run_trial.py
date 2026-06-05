#!/usr/bin/env python3
"""IKA Faz 0 — tek koşum harness (debug_world.sdf).

Senaryo: robot (0,0) → goal (3,0), tek engel obs_1 (1.5,0).

Beklenti: sim_full.launch.py + world:=debug_world ZATEN ayrı bir terminalde
çalışıyor olmalı. Bu script SADECE goal yayınlar + sonucu ölçer + CSV satırı
basar. Sim'i kendisi başlatmaz (repeat_trial.sh bunu yapar).

PASS koşulu  : |robot - goal| <= 0.50 m  VE  min_engel_mesafe > 0.25 m
FAIL_COLL    : min_engel_mesafe <= 0.25 m
FAIL_NAV     : Nav2 NavigateToPose action ABORTED/REJECTED
FAIL_TIMEOUT : süre doldu, hedefe varılmadı

Çıktı: TEK satır CSV, stdout'a:
  trial_id,status,final_x,final_y,dist_to_goal,min_obs_dist,duration,nav2_result

Kullanım:
  python3 run_trial.py --trial-id 1 --timeout 60
"""

import argparse
import csv
import math
import sys
import time
from io import StringIO

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose


# debug_world.sdf'teki sabit konfig
OBSTACLE_XY = (1.5, 0.0)
GOAL_XY = (3.0, 0.0)
COLLISION_THRESHOLD = 0.25   # m — bundan yakın = FAIL_COLL
GOAL_TOLERANCE = 0.50         # m — bundan yakın = goal'e vardı


class TrialMonitor(Node):
    def __init__(self):
        super().__init__('trial_monitor')

        # state
        self.odom_xy = None
        self.min_obs_dist = float('inf')
        self.collision_triggered = False
        self.t_start = None

        # subscribers
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        scan_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(LaserScan, '/scan', self._scan_cb, scan_qos)

        # nav2 action
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.odom_xy = (p.x, p.y)

    def _scan_cb(self, msg: LaserScan):
        # Robot pozundan engele uzaklık takip etmek yerine, lidar'ın en yakın
        # noktasını izleriz (collision_monitor mantığı). Sınır duvarları y=±2m
        # var; ama "engel" yakınlığını ölçmek için global frame'de obs_1 (1.5,0)
        # konumunu robotun pozuyla karşılaştırmak daha temiz.
        if self.odom_xy is None:
            return
        rx, ry = self.odom_xy
        ox, oy = OBSTACLE_XY
        d = math.hypot(rx - ox, ry - oy) - 0.20  # engel yarıçapı 0.20m
        if d < self.min_obs_dist:
            self.min_obs_dist = d
        if d <= COLLISION_THRESHOLD:
            self.collision_triggered = True

    def wait_for_sim_ready(self, timeout: float = 30.0) -> bool:
        """Odom akışı + Nav2 action server hazır olunca True."""
        t0 = time.time()
        # odom akışı
        while self.odom_xy is None and (time.time() - t0) < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.odom_xy is None:
            self.get_logger().error('Timeout: /odom akmadı')
            return False
        # nav2 server
        if not self.nav_client.wait_for_server(timeout_sec=timeout - (time.time() - t0)):
            self.get_logger().error('Timeout: /navigate_to_pose server gelmedi')
            return False
        return True

    def send_goal(self, x: float, y: float):
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.w = 1.0
        goal_msg.pose = pose
        self.t_start = time.time()
        self._send_future = self.nav_client.send_goal_async(goal_msg)
        return self._send_future

    def run_trial(self, timeout_s: float) -> dict:
        """Goal gönder + sonucu bekle. dict döner."""
        send_future = self.send_goal(*GOAL_XY)
        # send_goal accept'i bekle
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=5.0)
        if not send_future.done() or send_future.result() is None:
            return self._mkresult('FAIL_NAV', nav_result='goal_send_timeout')
        goal_handle = send_future.result()
        if not goal_handle.accepted:
            return self._mkresult('FAIL_NAV', nav_result='goal_rejected')

        result_future = goal_handle.get_result_async()

        # spin loop: timeout VEYA collision VEYA result hazır
        deadline = time.time() + timeout_s
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.collision_triggered:
                # iptal et + FAIL_COLL döndür
                goal_handle.cancel_goal_async()
                return self._mkresult('FAIL_COLL', nav_result='cancelled_by_collision')
            if result_future.done():
                break

        if not result_future.done():
            goal_handle.cancel_goal_async()
            return self._mkresult('FAIL_TIMEOUT', nav_result='timeout')

        status = result_future.result().status
        status_map = {
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_ABORTED:   'ABORTED',
            GoalStatus.STATUS_CANCELED:  'CANCELED',
        }
        nav_result = status_map.get(status, f'status_{status}')

        # Nav2 SUCCEEDED dese bile, gerçek pozisyona bak
        if self.odom_xy is None:
            return self._mkresult('FAIL_NAV', nav_result=nav_result)
        dx = self.odom_xy[0] - GOAL_XY[0]
        dy = self.odom_xy[1] - GOAL_XY[1]
        dist = math.hypot(dx, dy)

        if self.collision_triggered:
            return self._mkresult('FAIL_COLL', nav_result=nav_result)
        if dist <= GOAL_TOLERANCE and status == GoalStatus.STATUS_SUCCEEDED:
            return self._mkresult('PASS', nav_result=nav_result)
        if status == GoalStatus.STATUS_ABORTED:
            return self._mkresult('FAIL_NAV', nav_result=nav_result)
        return self._mkresult('FAIL_NAV', nav_result=nav_result)

    def _mkresult(self, status: str, nav_result: str = '') -> dict:
        rx, ry = self.odom_xy if self.odom_xy else (float('nan'), float('nan'))
        dist = math.hypot(rx - GOAL_XY[0], ry - GOAL_XY[1]) if self.odom_xy else float('nan')
        duration = (time.time() - self.t_start) if self.t_start else 0.0
        return {
            'status': status,
            'final_x': rx,
            'final_y': ry,
            'dist_to_goal': dist,
            'min_obs_dist': self.min_obs_dist if math.isfinite(self.min_obs_dist) else -1.0,
            'duration': duration,
            'nav2_result': nav_result,
        }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trial-id', type=int, required=True)
    ap.add_argument('--timeout', type=float, default=60.0)
    ap.add_argument('--ready-timeout', type=float, default=45.0)
    args = ap.parse_args()

    rclpy.init()
    node = TrialMonitor()

    if not node.wait_for_sim_ready(timeout=args.ready_timeout):
        result = {
            'status': 'FAIL_NOT_READY', 'final_x': float('nan'),
            'final_y': float('nan'), 'dist_to_goal': float('nan'),
            'min_obs_dist': -1.0, 'duration': 0.0,
            'nav2_result': 'sim_not_ready',
        }
    else:
        result = node.run_trial(args.timeout)

    node.destroy_node()
    rclpy.shutdown()

    buf = StringIO()
    writer = csv.writer(buf)
    writer.writerow([
        args.trial_id, result['status'],
        f"{result['final_x']:.3f}", f"{result['final_y']:.3f}",
        f"{result['dist_to_goal']:.3f}", f"{result['min_obs_dist']:.3f}",
        f"{result['duration']:.2f}", result['nav2_result'],
    ])
    sys.stdout.write(buf.getvalue())
    sys.stdout.flush()
    # FAIL_* -> exit 1, PASS -> exit 0 (bash kolay parse)
    sys.exit(0 if result['status'] == 'PASS' else 1)


if __name__ == '__main__':
    main()
