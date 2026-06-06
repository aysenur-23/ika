"""IKA — Goal-Aware Reaktif Engel Kaçınma ROS Node sarmalı.

Defense-in-depth Katman 2 (`docs/avoidance_architecture.md`).

Topics:
    Sub:
      /scan                  sensor_msgs/LaserScan       (lidar)
      /odometry/filtered     nav_msgs/Odometry           (EKF, yaw)
      /odom                  nav_msgs/Odometry           (lidar odom, fallback)
      /detected_objects      vision_msgs/Detection3DArray (camera DL)
      /hazard_state          std_msgs/String              (fusion, IGNORED)
    Pub:
      /cmd_vel_nav           geometry_msgs/Twist          (collision_monitor girişi)
      /avoider_state         std_msgs/String              (debug, faz + heading)
"""
from __future__ import annotations

import json
import math
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

try:
    from vision_msgs.msg import Detection3DArray
    HAS_VISION_MSGS = True
except ImportError:
    HAS_VISION_MSGS = False

from ika_mission.avoider_logic import (
    AvoiderConfig, AvoiderState, AvoiderPhase, decide,
    side_minima, wrap_pi,
)


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class ObstacleAvoiderNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoider')

        self.declare_parameter('forward_speed_mps', 0.25)
        self.declare_parameter('turn_speed_rps', 0.5)
        self.declare_parameter('obstacle_distance_m', 0.35)
        self.declare_parameter('release_distance_m', 0.60)
        self.declare_parameter('camera_detection_distance_m', 0.50)
        self.declare_parameter('pass_clear_distance_m', 0.40)
        self.declare_parameter('front_arc_deg', 50.0)
        self.declare_parameter('target_distance_m', 10000.0)
        self.declare_parameter('yaw_tolerance_rad', 0.15)
        # KULLANICI fix: DRIVING heading correction kapatildi (sacma sapma)
        self.declare_parameter('heading_kp', 0.0)
        self.declare_parameter('max_heading_correction_rps', 0.0)
        self.declare_parameter('heading_critical_err_rad', 9999.0)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('start_delay_s', 3.0)

        self._cfg = AvoiderConfig(
            forward_speed_mps=float(self.get_parameter('forward_speed_mps').value),
            turn_speed_rps=float(self.get_parameter('turn_speed_rps').value),
            obstacle_distance_m=float(self.get_parameter('obstacle_distance_m').value),
            release_distance_m=float(self.get_parameter('release_distance_m').value),
            camera_detection_distance_m=float(self.get_parameter('camera_detection_distance_m').value),
            pass_clear_distance_m=float(self.get_parameter('pass_clear_distance_m').value),
            front_arc_deg=float(self.get_parameter('front_arc_deg').value),
            target_distance_m=float(self.get_parameter('target_distance_m').value),
            yaw_tolerance_rad=float(self.get_parameter('yaw_tolerance_rad').value),
            heading_kp=float(self.get_parameter('heading_kp').value),
            max_heading_correction_rps=float(self.get_parameter('max_heading_correction_rps').value),
            heading_critical_err_rad=float(self.get_parameter('heading_critical_err_rad').value),
        )
        self._state: Optional[AvoiderState] = None
        self._last_scan: Optional[LaserScan] = None
        self._last_hazard_action: str = "CLEAR"
        self._current_yaw: float = 0.0
        self._initial_yaw: Optional[float] = None
        # Gerçek odom pozisyonu — delta hesaplamak için
        self._last_position: Optional[tuple] = None
        self._current_position: Optional[tuple] = None
        # Camera DL detection — en yakın engel mesafesi + sınıfı
        self._camera_obstacle_dist: float = float('inf')
        self._camera_active_class: str = "none"
        self._start_time = time.time()
        self._start_delay = float(self.get_parameter('start_delay_s').value)

        qos_sensor = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_sensor)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        # hazard_state hala dinleniyor ama avoider_logic IGNORE eder (debug için)
        self.create_subscription(String, '/hazard_state', self._on_hazard, 10)

        # Camera DL detection — vision_msgs varsa sub
        if HAS_VISION_MSGS:
            self.create_subscription(
                Detection3DArray, '/detected_objects',
                self._on_detection, 10)
            self.get_logger().info("Camera DL detection: /detected_objects abonesi aktif")
        else:
            self.get_logger().warn("vision_msgs yok — camera detection devre dışı")

        self._pub_cmd = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self._pub_state = self.create_publisher(String, '/avoider_state', 10)
        # Telemetri: sade state + zengin debug
        self._pub_state_short = self.create_publisher(String, '/avoider/state', 10)
        self._pub_debug = self.create_publisher(String, '/avoider/debug', 10)

        rate = float(self.get_parameter('control_rate_hz').value)
        self._period = 1.0 / max(rate, 1.0)
        self.create_timer(self._period, self._tick)
        self.get_logger().info(
            f"Goal-aware avoider ready; start_delay={self._start_delay}s, "
            f"speed={self._cfg.forward_speed_mps}m/s, "
            f"obstacle@{self._cfg.obstacle_distance_m}m, "
            f"camera@{self._cfg.camera_detection_distance_m}m")

    def _on_scan(self, msg: LaserScan):
        self._last_scan = msg

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self._current_yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
        p = msg.pose.pose.position
        self._current_position = (p.x, p.y)

    def _on_hazard(self, msg: String):
        try:
            payload = json.loads(msg.data)
            self._last_hazard_action = str(payload.get('action', 'CLEAR'))
        except (ValueError, TypeError):
            self._last_hazard_action = 'CLEAR'

    def _on_detection(self, msg):
        """Camera DL detection — en yakın engelin uzaklığını al."""
        if not msg.detections:
            self._camera_obstacle_dist = float('inf')
            self._camera_active_class = "none"
            return
        min_d = float('inf')
        active_class = "none"
        for det in msg.detections:
            # bbox center pozisyonu (camera frame veya base_link frame)
            # Detection3D.bbox.center.position.{x,y,z}
            try:
                p = det.bbox.center.position
                d = math.hypot(p.x, p.y)  # x ileri, y yan
                # Sadece ÖN sektördekileri dikkate al (x > 0 ve dar yan sapma)
                if p.x > 0 and abs(p.y) < 1.5:
                    if d < min_d:
                        min_d = d
                        try:
                            active_class = str(det.results[0].hypothesis.class_id) \
                                if det.results else "none"
                        except (AttributeError, IndexError):
                            active_class = "none"
            except AttributeError:
                continue
        self._camera_obstacle_dist = min_d
        self._camera_active_class = active_class

    def _tick(self):
        if (time.time() - self._start_time) < self._start_delay:
            return

        # State'i ilk tikte initialize et — INITIAL PATH PLANNING
        if self._state is None:
            # Robot başlangıçta hangi yöne bakıyorsa, hedef o yön
            self._initial_yaw = self._current_yaw
            self._state = AvoiderState(
                phase=AvoiderPhase.DRIVING,
                goal_heading_rad=self._current_yaw,
                distance_clear_m=0.0,
                avoid_direction=0,
                pass_distance_m=0.0,
            )
            self.get_logger().info(
                f"Initial plan: goal_heading={self._current_yaw:.3f} rad "
                f"({math.degrees(self._current_yaw):.1f}°)")

        scan = self._last_scan
        if scan is None:
            return

        # Gerçek odom-based delta
        odom_delta_m = 0.0
        if self._current_position is not None and self._last_position is not None:
            dx = self._current_position[0] - self._last_position[0]
            dy = self._current_position[1] - self._last_position[1]
            odom_delta_m = math.hypot(dx, dy)
        if self._current_position is not None:
            self._last_position = self._current_position

        total_fov = scan.angle_max - scan.angle_min
        cmd = decide(
            state=self._state,
            scan_ranges=scan.ranges,
            scan_fov_rad=total_fov,
            hazard_action=self._last_hazard_action,
            current_yaw=self._current_yaw,
            odom_delta_m=odom_delta_m,
            cfg=self._cfg,
            camera_obstacle_distance_m=self._camera_obstacle_dist,
        )
        self._state = cmd.next_state

        m = Twist()
        m.linear.x = float(cmd.linear_x)
        m.angular.z = float(cmd.angular_z)
        self._pub_cmd.publish(m)

        # ── Telemetri hesapları ─────────────────────────────────────────
        front_arc_rad = math.radians(self._cfg.front_arc_deg)
        f_min, l_min, r_min = side_minima(
            scan.ranges,
            scan.angle_min,
            scan.angle_increment,
            front_arc_rad,
        )
        yaw_err = wrap_pi(self._state.goal_heading_rad - self._current_yaw)
        cam_d = self._camera_obstacle_dist
        cam_detected = math.isfinite(cam_d) and cam_d < self._cfg.camera_detection_distance_m
        obstacle_detected = (math.isfinite(f_min) and
                             f_min < self._cfg.obstacle_distance_m) or cam_detected
        # chosen_side string: "left" / "right" / "none"
        _ad = int(self._state.avoid_direction)
        chosen_side = "left" if _ad > 0 else ("right" if _ad < 0 else "none")

        def _safe(x, default=-1.0):
            try:
                xf = float(x)
            except (TypeError, ValueError):
                return default
            return round(xf, 3) if math.isfinite(xf) else default

        # Geriye uyumlu zengin state (eski tüketiciler için aynen kalsın)
        lidar_min = min((r for r in scan.ranges
                         if isinstance(r, (int, float)) and r > 0 and math.isfinite(r)),
                        default=float('inf'))
        s = String()
        s.data = json.dumps({
            "phase": str(self._state.phase.value),
            "distance_clear_m": _safe(self._state.distance_clear_m, 0.0),
            "goal_heading": _safe(self._state.goal_heading_rad, 0.0),
            "current_yaw": _safe(self._current_yaw, 0.0),
            "yaw_err": _safe(yaw_err, 0.0),
            "avoid_direction": _ad,
            "lidar_min_obs": _safe(lidar_min),
            "camera_min_obs": _safe(cam_d),
            "hazard_action": self._last_hazard_action,
            "reason": cmd.reason,
        }, allow_nan=False)
        self._pub_state.publish(s)

        # Sade state — tek string, kolay loglama / RViz plot
        ss = String()
        ss.data = str(self._state.phase.value)
        self._pub_state_short.publish(ss)

        # Zengin debug — TASK-1 alanları
        dbg = String()
        dbg.data = json.dumps({
            "state": str(self._state.phase.value),
            "front_min": _safe(f_min),
            "left_min": _safe(l_min),
            "right_min": _safe(r_min),
            "chosen_side": chosen_side,
            "obstacle_detected": bool(obstacle_detected),
            "camera_detected": bool(cam_detected),
            "active_class": self._camera_active_class,
            "current_yaw": _safe(self._current_yaw, 0.0),
            "target_yaw": _safe(self._state.goal_heading_rad, 0.0),
            "yaw_error": _safe(yaw_err, 0.0),
            "cmd_linear": _safe(cmd.linear_x, 0.0),
            "cmd_angular": _safe(cmd.angular_z, 0.0),
        }, allow_nan=False)
        self._pub_debug.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoiderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
