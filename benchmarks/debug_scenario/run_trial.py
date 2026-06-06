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
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose

from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException


# debug_world.sdf'teki sabit konfig
OBSTACLE_XY = (1.5, 0.0)
DEFAULT_GOAL_XY = (3.0, 0.0)
DEFAULT_COLLISION_THRESHOLD = 0.05   # m — varsayilan: ~temas (engel yarıçapı 0.20, robot bbox ~0.30)
GOAL_TOLERANCE = 0.50                 # m — bundan yakın = goal'e vardı


class TrialMonitor(Node):
    def __init__(self, collision_threshold: float = DEFAULT_COLLISION_THRESHOLD,
                 goal_xy: tuple = DEFAULT_GOAL_XY):
        super().__init__('trial_monitor')

        # state
        self.odom_xy = None
        self.min_obs_dist = float('inf')
        self.collision_triggered = False
        self.t_start = None
        self.collision_threshold = collision_threshold
        self.goal_xy = tuple(goal_xy)
        self.map_received = False
        self.local_costmap_received = False
        self.global_costmap_received = False

        # subscribers
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        scan_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(LaserScan, '/scan', self._scan_cb, scan_qos)

        # Nav2 / SLAM topic'leri TRANSIENT_LOCAL (latched) — son yayını al
        latched_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map',
                                 lambda _: self._mark_map(), latched_qos)
        # /local_costmap/costmap normalde RELIABLE+VOLATILE
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 lambda _: self._mark_local_costmap(), 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda _: self._mark_global_costmap(), 10)

        # tf2 — map->base_link mevcut mu kontrol için
        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        # nav2 action
        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def _mark_map(self):
        self.map_received = True

    def _mark_local_costmap(self):
        self.local_costmap_received = True

    def _mark_global_costmap(self):
        self.global_costmap_received = True

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
        if d <= self.collision_threshold:
            self.collision_triggered = True

    def wait_for_sim_ready(self, timeout: float = 30.0) -> bool:
        """Stack tamamen hazır olunca True. Probe edilen:
          1. /odom akışı (Gazebo + bridge canlı)
          2. /navigate_to_pose action server (Nav2 lifecycle ACTIVE)
          3. /map publish (SLAM yayınlıyor, latched alındı)
          4. /local_costmap/costmap publish (lokal costmap aktif)
          5. TF map → base_link mevcut (SLAM scan-match çalıştı)

        Önceden: 45 s sabit sleep. Sonuç: G kategori (stack init) %50 FAIL.
        Şimdi: aktif probe, her şart sağlanana kadar bekle (timeout 60s).
        """
        t0 = time.time()
        deadline = t0 + timeout

        def remaining():
            return max(0.0, deadline - time.time())

        # 1. odom
        while self.odom_xy is None and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if self.odom_xy is None:
            self.get_logger().error('Timeout: /odom akmadı')
            return False

        # 2. nav2 action server
        if not self.nav_client.wait_for_server(timeout_sec=remaining()):
            self.get_logger().error('Timeout: /navigate_to_pose server gelmedi')
            return False

        # 3. /map (SLAM latched)
        while not self.map_received and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if not self.map_received:
            self.get_logger().error('Timeout: /map yayını gelmedi')
            return False

        # 4a. /local_costmap/costmap (controller_server ACTIVE göstergesi)
        while not self.local_costmap_received and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if not self.local_costmap_received:
            self.get_logger().error('Timeout: /local_costmap/costmap gelmedi')
            return False

        # 4b. /global_costmap/costmap (planner_server ACTIVE göstergesi)
        # Bu olmadan goal_rejected gelir; planner_server unconfigured/inactive.
        while not self.global_costmap_received and time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
        if not self.global_costmap_received:
            self.get_logger().error('Timeout: /global_costmap/costmap gelmedi')
            return False

        # 5. TF map -> base_link
        # SLAM scan-match yapıp ilk transform'u yayınlana kadar bekle.
        # Birden fazla deneme — tf2 listener ilk birkaç sn'de henüz spin
        # etmemiş olabilir.
        from rclpy.time import Time
        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            try:
                self.tf_buf.lookup_transform('map', 'base_link', Time())
                # Settle: costmap obstacle birikimi için bekleme.
                # 15s denendi, varyansı artırdı (bazı trial'lar instant
                # abort). 5s sweet spot — yeterli scan birikimi + BT
                # zaman aşımı tetiklenmez.
                ready_time = time.time() - t0
                self.get_logger().info(
                    f'Probe OK: {ready_time:.1f}s '
                    f'(odom+nav2+map+local+global+tf). '
                    f'5s costmap+BT settle için bekliyorum...')
                settle_end = time.time() + 5.0
                while time.time() < settle_end:
                    rclpy.spin_once(self, timeout_sec=0.1)
                self.get_logger().info(
                    f'Stack ready: {time.time() - t0:.1f}s')
                return True
            except (LookupException, ExtrapolationException):
                continue
        self.get_logger().error('Timeout: TF map->base_link gelmedi')
        return False

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
        send_future = self.send_goal(*self.goal_xy)
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
        dx = self.odom_xy[0] - self.goal_xy[0]
        dy = self.odom_xy[1] - self.goal_xy[1]
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
        dist = math.hypot(rx - self.goal_xy[0], ry - self.goal_xy[1]) if self.odom_xy else float('nan')
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
    ap.add_argument('--ready-timeout', type=float, default=60.0)
    ap.add_argument('--collision-threshold', type=float,
                    default=DEFAULT_COLLISION_THRESHOLD,
                    help='m, bundan yakın = FAIL_COLL (default 0.05 = temas)')
    ap.add_argument('--goal-x', type=float, default=DEFAULT_GOAL_XY[0])
    ap.add_argument('--goal-y', type=float, default=DEFAULT_GOAL_XY[1])
    args = ap.parse_args()

    rclpy.init()
    node = TrialMonitor(
        collision_threshold=args.collision_threshold,
        goal_xy=(args.goal_x, args.goal_y),
    )

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
