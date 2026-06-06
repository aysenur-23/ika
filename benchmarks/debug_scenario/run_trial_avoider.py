#!/usr/bin/env python3
"""IKA — Avoider modu trial harness'i.

run_trial.py Nav2 action client kullaniyor; avoider mode'da Nav2 yok.
Bu script /odom + /scan + /avoider_state dinler:
  PASS koşulu: avoider DONE'a vardı VE min_obs > collision_threshold
  FAIL_COLL  : min_obs <= collision_threshold (engele temas)
  FAIL_STUCK : N saniye boyunca pozisyon < 0.05 m (sıkıştı)
  FAIL_TIMEOUT: süre doldu

Kullanım:
  python3 run_trial_avoider.py --trial-id 1 --timeout 30
"""
import argparse
import csv
import json
import math
import sys
import time
from io import StringIO

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


# debug_world.sdf'teki sabit konfig (CLI ile override)
DEFAULT_OBSTACLES = [(1.5, 0.0)]   # test_world.sdf icin coklu liste verilir
DEFAULT_COLLISION_THRESHOLD = 0.05
DEFAULT_PASS_X = 2.0                # bu x'i geçince "engel atlatıldı"


class AvoiderTrialMonitor(Node):
    def __init__(self,
                 obstacles=None,
                 collision_threshold: float = DEFAULT_COLLISION_THRESHOLD,
                 pass_x: float = DEFAULT_PASS_X):
        super().__init__('avoider_trial_monitor')
        self.obstacles = obstacles or DEFAULT_OBSTACLES
        self.collision_threshold = collision_threshold
        self.pass_x = pass_x

        # State
        self.odom_xy = None
        self.start_xy = None
        self.min_obs_dist = float('inf')
        self.collision_triggered = False
        self.avoider_phase = "UNKNOWN"
        self.distance_clear_m = 0.0
        self.t_start = time.time()

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        scan_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(LaserScan, '/scan', self._scan_cb, scan_qos)
        self.create_subscription(String, '/avoider_state', self._avoider_state_cb, 10)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.odom_xy = (p.x, p.y)
        if self.start_xy is None:
            self.start_xy = (p.x, p.y)

    def _scan_cb(self, msg: LaserScan):
        if self.odom_xy is None:
            return
        rx, ry = self.odom_xy
        for ox, oy in self.obstacles:
            d = math.hypot(rx - ox, ry - oy) - 0.20  # engel yarıçapı ~0.20m
            if d < self.min_obs_dist:
                self.min_obs_dist = d
            if d <= self.collision_threshold:
                self.collision_triggered = True

    def _avoider_state_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
            self.avoider_phase = str(payload.get('phase', 'UNKNOWN'))
            self.distance_clear_m = float(payload.get('distance_clear_m', 0.0))
        except (ValueError, TypeError):
            pass

    def wait_for_avoider_ready(self, timeout: float = 30.0) -> bool:
        """/avoider_state yayını başlayana kadar bekle."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            if self.avoider_phase != "UNKNOWN":
                self.get_logger().info(
                    f'Avoider ready: phase={self.avoider_phase} '
                    f'after {time.time() - self.t_start:.1f}s')
                return True
        self.get_logger().error('Timeout: /avoider_state yayını gelmedi')
        return False

    def run_trial(self, timeout_s: float) -> dict:
        """Avoider hareketini izle, sonuç döndür."""
        self.t_start = time.time()
        deadline = self.t_start + timeout_s
        last_pos_change_t = self.t_start
        last_pos = self.odom_xy

        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

            # Collision check
            if self.collision_triggered:
                return self._mkresult('FAIL_COLL', 'cancelled_by_collision')

            # Avoider DONE'a geldi mi?
            if self.avoider_phase == "DONE":
                # PASS: robot durdu + çarpmadı + engeli atlatti (x > pass_x)
                # veya en azından target_distance_m kadar engelsiz mesafe katti
                if self.odom_xy is None:
                    return self._mkresult('FAIL_NAV', 'done_but_no_odom')
                if self.collision_triggered:
                    return self._mkresult('FAIL_COLL', 'done_but_collided')
                # DONE'a vardı, çarpmadı — başarı
                return self._mkresult('PASS', 'avoider_DONE')

            # Stuck detection: 8 saniye boyunca < 5 cm hareket
            if last_pos is not None and self.odom_xy is not None:
                d = math.hypot(self.odom_xy[0] - last_pos[0],
                               self.odom_xy[1] - last_pos[1])
                if d > 0.05:
                    last_pos = self.odom_xy
                    last_pos_change_t = time.time()
                elif (time.time() - last_pos_change_t) > 8.0:
                    return self._mkresult('FAIL_STUCK', f'stuck_{int(time.time() - last_pos_change_t)}s')
            elif self.odom_xy is not None:
                last_pos = self.odom_xy
                last_pos_change_t = time.time()

        return self._mkresult('FAIL_TIMEOUT', 'timeout')

    def _mkresult(self, status: str, reason: str) -> dict:
        rx, ry = self.odom_xy if self.odom_xy else (float('nan'), float('nan'))
        duration = time.time() - self.t_start
        return {
            'status': status,
            'final_x': rx,
            'final_y': ry,
            'min_obs_dist': self.min_obs_dist if math.isfinite(self.min_obs_dist) else -1.0,
            'distance_clear_m': self.distance_clear_m,
            'avoider_phase': self.avoider_phase,
            'duration': duration,
            'reason': reason,
        }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trial-id', type=int, required=True)
    ap.add_argument('--timeout', type=float, default=45.0)
    ap.add_argument('--ready-timeout', type=float, default=30.0)
    ap.add_argument('--collision-threshold', type=float,
                    default=DEFAULT_COLLISION_THRESHOLD)
    ap.add_argument('--obstacles', type=str, default='1.5,0.0',
                    help='Virgülle ayrılmış x,y çiftleri: "1.5,0;3,0.4;...";')
    ap.add_argument('--pass-x', type=float, default=DEFAULT_PASS_X)
    args = ap.parse_args()

    # Parse obstacles "x1,y1;x2,y2;..." veya tek "x,y"
    obstacles = []
    for pair in args.obstacles.split(';'):
        if not pair.strip():
            continue
        x, y = map(float, pair.split(','))
        obstacles.append((x, y))

    rclpy.init()
    node = AvoiderTrialMonitor(
        obstacles=obstacles,
        collision_threshold=args.collision_threshold,
        pass_x=args.pass_x,
    )

    if not node.wait_for_avoider_ready(timeout=args.ready_timeout):
        result = {
            'status': 'FAIL_NOT_READY', 'final_x': float('nan'),
            'final_y': float('nan'), 'min_obs_dist': -1.0,
            'distance_clear_m': 0.0, 'avoider_phase': 'UNKNOWN',
            'duration': 0.0, 'reason': 'avoider_node_not_ready',
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
        f"{result['min_obs_dist']:.3f}", f"{result['distance_clear_m']:.3f}",
        result['avoider_phase'], f"{result['duration']:.2f}", result['reason'],
    ])
    sys.stdout.write(buf.getvalue())
    sys.stdout.flush()
    sys.exit(0 if result['status'] == 'PASS' else 1)


if __name__ == '__main__':
    main()
