"""IKA - Planlayici karsilastirma metrik kaydedici.

Sim'de DWB ile MPPI (ileride ogrenilmis politika) kosumlarini ayni olcutlerle
karsilastirmak icin. /goal_pose geldiginde kayit baslar; robot hedefe ulasinca
(veya zaman asiminda) kosum sonlanir, planner_metrics ile ozetlenir ve CSV'ye
eklenir. Boylece `local_planner:=dwb` ve `local_planner:=mppi` kosumlari ayni
dosyada satir satir karsilastirilir.

  /goal_pose            (geometry_msgs/PoseStamped) -> kosum baslat
  /odometry/filtered    (nav_msgs/Odometry)         -> yorunge

Cikti: output_csv'ye bir satir/kosum (planner, success, sure, yol, clearance...).
"""
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

from ika_rl_planner.planner_metrics import summarize_run


class MetricsRecorderNode(Node):

    def __init__(self):
        super().__init__('planner_metrics_recorder')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('output_csv', 'planner_comparison.csv')
        self.declare_parameter('planner_label', 'dwb')
        self.declare_parameter('goal_tolerance_m', 0.25)
        # Bilinen engel konumlari (odom/map frame), duz liste [x1,y1,x2,y2,...]
        self.declare_parameter('obstacles', [0.0])
        # Hedefe ulasilamazsa kosumu bitir (saniye)
        self.declare_parameter('run_timeout_s', 120.0)

        self._points = []
        self._goal = None
        self._start_t = None
        self._active = False

        self.create_subscription(
            Odometry, self.get_parameter('odom_topic').value,
            self._on_odom, 20)
        self.create_subscription(
            PoseStamped, self.get_parameter('goal_topic').value,
            self._on_goal, 10)
        self.create_timer(0.5, self._check)

        self.get_logger().info(
            f"Metrik kaydedici hazir (planner={self.get_parameter('planner_label').value}). "
            "Bir /goal_pose yayinlayin.")

    def _obstacles(self):
        flat = list(self.get_parameter('obstacles').value)
        if len(flat) < 2:
            return []
        return [(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]

    def _on_goal(self, msg: PoseStamped):
        self._goal = (msg.pose.position.x, msg.pose.position.y)
        self._points = []
        self._start_t = time.time()
        self._active = True
        self.get_logger().info(f'Kosum basladi -> hedef {self._goal}')

    def _on_odom(self, msg: Odometry):
        if not self._active:
            return
        p = msg.pose.pose.position
        self._points.append((p.x, p.y))

    def _check(self):
        if not self._active or self._goal is None or not self._points:
            return
        tol = float(self.get_parameter('goal_tolerance_m').value)
        elapsed = time.time() - self._start_t
        fx, fy = self._points[-1]
        reached = math.hypot(fx - self._goal[0], fy - self._goal[1]) <= tol
        timed_out = elapsed > float(self.get_parameter('run_timeout_s').value)
        if reached or timed_out:
            self._finalize(elapsed)

    def _finalize(self, elapsed: float):
        self._active = False
        result = summarize_run(
            self._points,
            duration_s=elapsed,
            goal_xy=self._goal,
            obstacles=self._obstacles(),
            goal_tolerance_m=float(self.get_parameter('goal_tolerance_m').value),
        )
        label = self.get_parameter('planner_label').value
        path = self.get_parameter('output_csv').value
        self._append_csv(path, label, result)
        self.get_logger().info(
            f'Kosum bitti [{label}] success={result.success} '
            f'sure={result.duration_s}s yol={result.path_length_m}m '
            f'clearance={result.min_clearance_m}m egrilik={result.mean_abs_curvature}')

    def _append_csv(self, path: str, label: str, result):
        header = ['timestamp', 'planner', 'success', 'duration_s',
                  'path_length_m', 'min_clearance_m', 'mean_abs_curvature',
                  'avg_speed_mps', 'num_points']
        exists = os.path.isfile(path)
        try:
            with open(path, 'a', newline='') as f:
                w = csv.writer(f)
                if not exists:
                    w.writerow(header)
                d = result.as_dict()
                w.writerow([
                    time.strftime('%Y-%m-%d %H:%M:%S'), label,
                    d['success'], d['duration_s'], d['path_length_m'],
                    d['min_clearance_m'], d['mean_abs_curvature'],
                    d['avg_speed_mps'], d['num_points'],
                ])
        except OSError as exc:
            self.get_logger().error(f'CSV yazilamadi ({path}): {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = MetricsRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
