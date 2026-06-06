"""IKA — Reaktif engel kacinma ROS plain Node sarmali.

NOT: Onceden LifecycleNode idi; lifecycle_manager bond timeout sonrasi
re-configure denemesinde "transition not registered" hatasi veriyordu (rclpy
bug). Plain Node'a indirildi — on init aktif olarak baslar. Kullanici spec'i:
"guc geldiginde dumduz baslar" — auto-start tam bunu yapar.

Topics:
    Sub:
      /scan                  sensor_msgs/LaserScan
      /odometry/filtered     nav_msgs/Odometry
      /hazard_state          std_msgs/String        (JSON, fusion ciktisi)
    Pub:
      /cmd_vel_nav           geometry_msgs/Twist    (collision_monitor girisi)
      /avoider_state         std_msgs/String        (faz + reason, debug)
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

from ika_mission.avoider_logic import (
    AvoiderConfig, AvoiderState, AvoiderPhase, decide,
)


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class ObstacleAvoiderNode(Node):

    def __init__(self):
        super().__init__('obstacle_avoider')

        self.declare_parameter('forward_speed_mps', 0.20)
        self.declare_parameter('turn_speed_rps', 0.5)
        self.declare_parameter('obstacle_distance_m', 0.80)
        self.declare_parameter('front_arc_deg', 60.0)
        self.declare_parameter('target_distance_m', 2.0)
        self.declare_parameter('yaw_tolerance_rad', 0.05)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('start_delay_s', 3.0)

        # Yeni parametreler: PASSING + hysteresis (avoider_logic v2)
        self.declare_parameter('release_distance_m', 1.00)
        self.declare_parameter('pass_clear_distance_m', 0.50)

        self._cfg = AvoiderConfig(
            forward_speed_mps=float(self.get_parameter('forward_speed_mps').value),
            turn_speed_rps=float(self.get_parameter('turn_speed_rps').value),
            obstacle_distance_m=float(self.get_parameter('obstacle_distance_m').value),
            release_distance_m=float(self.get_parameter('release_distance_m').value),
            pass_clear_distance_m=float(self.get_parameter('pass_clear_distance_m').value),
            front_arc_deg=float(self.get_parameter('front_arc_deg').value),
            target_distance_m=float(self.get_parameter('target_distance_m').value),
            yaw_tolerance_rad=float(self.get_parameter('yaw_tolerance_rad').value),
        )
        self._state: Optional[AvoiderState] = None
        self._last_scan: Optional[LaserScan] = None
        self._last_hazard_action: str = "CLEAR"
        self._current_yaw: float = 0.0
        # Gerçek odom pozisyonu — delta hesaplamak için
        self._last_position: Optional[tuple] = None
        self._current_position: Optional[tuple] = None
        self._start_time = time.time()
        self._start_delay = float(self.get_parameter('start_delay_s').value)

        qos_sensor = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_sensor)
        # /odom (rf2o lidar odom) — EKF /odometry/filtered yayinlamiyorsa
        # bile bu akar. Avoider yaw'i icin yeterli.
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(String, '/hazard_state', self._on_hazard, 10)

        self._pub_cmd = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self._pub_state = self.create_publisher(String, '/avoider_state', 10)

        rate = float(self.get_parameter('control_rate_hz').value)
        self._period = 1.0 / max(rate, 1.0)
        self.create_timer(self._period, self._tick)
        self.get_logger().info(
            f"Obstacle avoider ready; start_delay={self._start_delay}s, "
            f"speed={self._cfg.forward_speed_mps}m/s")

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

    def _tick(self):
        # Baslangic gecikmesi — gz sim + bridge ayaga kalksin
        if (time.time() - self._start_time) < self._start_delay:
            return
        if self._state is None:
            self._state = AvoiderState(
                phase=AvoiderPhase.DRIVING,
                home_yaw=self._current_yaw,
                distance_clear_m=0.0,
                avoid_direction=0,
            )
            self.get_logger().info(
                f"Driving forward (home_yaw={self._current_yaw:.3f})")

        scan = self._last_scan
        if scan is None:
            return

        # Gerçek odom-based delta (DRIVING ve PASSING fazlarında hareket var)
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
        )
        self._state = cmd.next_state

        m = Twist()
        m.linear.x = float(cmd.linear_x)
        m.angular.z = float(cmd.angular_z)
        self._pub_cmd.publish(m)

        s = String()
        s.data = json.dumps({
            "phase": str(self._state.phase.value),
            "distance_clear_m": round(self._state.distance_clear_m, 3),
            "home_yaw": round(self._state.home_yaw, 3),
            "current_yaw": round(self._current_yaw, 3),
            "avoid_direction": int(self._state.avoid_direction),
            "hazard_action": self._last_hazard_action,
            "reason": cmd.reason,
        })
        self._pub_state.publish(s)


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
