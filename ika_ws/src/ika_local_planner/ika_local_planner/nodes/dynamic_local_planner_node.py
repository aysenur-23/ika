"""IKA — Dynamic Local Planner ROS node.

Thin ROS wrapper around the pure-Python core:
    local_costmap + semantic_policy + local_planner_logic + path_rejoin.

Benchmark uyumluluğu (TASK-3.1 harness'i değişmesin):
    Subs : /scan, /odom, /detected_objects (opsiyonel)
    Pubs : /cmd_vel_nav, /avoider/debug, /avoider/state, /avoider_state
    Srvs : /avoider/start, /avoider/stop  (Trigger)

Bu fazda (TASK-4B-1) tek node, basit kontrolör, reflex stop. A*/DWA yok.
"""
from __future__ import annotations

import json
import math
import time
from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from vision_msgs.msg import Detection3DArray
    HAS_VISION_MSGS = True
except ImportError:
    HAS_VISION_MSGS = False

from ika_local_planner.local_costmap import (
    CostmapConfig, Detection, LocalCostmap,
    build_costmap_from_scan, overlay_detections,
    summarize_costmap, find_free_lateral_gaps,
)
from ika_local_planner.semantic_policy import (
    BehaviorMode, BehaviorDecision, select_behavior,
)
from ika_local_planner.local_planner_logic import (
    Pose2D, Waypoint, PlannerConfig, LocalPlan,
    plan_local_waypoint,
)
# path_rejoin doğrudan kullanılmıyor (planner zaten merkez offset'i tercih
# eder); rejoin_required flag debug'a yansıtılır. TASK-4B-2'de devreye girer.

# Default semantic weights (sim sınıfları). Bilinmeyen → unknown_cost.
_DEFAULT_SEMANTIC_WEIGHTS = {
    'person': 0.95,
    'pedestrian': 0.95,
    'box': 0.9,
    'obstacle': 0.9,
    'pole': 0.85,
    'cone': 0.8,
    'pothole': 0.7,
    'pit': 0.8,
    'threshold': 0.6,
    'ramp': 0.4,
    'wall': 0.9,
    'corridor': 0.8,
    'ground_patch': 0.2,
}


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _clip(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def _safe(x, default=-1.0):
    try:
        xf = float(x)
    except (TypeError, ValueError):
        return default
    return round(xf, 3) if math.isfinite(xf) else default


class DynamicLocalPlannerNode(Node):

    def __init__(self):
        super().__init__('dynamic_local_planner')

        # ── Parametreler ──────────────────────────────────────────────
        self.declare_parameter('auto_start', False)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('target_x', 22.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('path_y', 0.0)
        self.declare_parameter('target_heading_rad', 0.0)
        self.declare_parameter('default_speed_mps', 0.22)
        self.declare_parameter('slow_speed_mps', 0.12)
        self.declare_parameter('max_angular_rps', 0.55)
        self.declare_parameter('kp_yaw', 1.3)
        self.declare_parameter('reflex_stop_distance_m', 0.20)
        self.declare_parameter('safety_cost_threshold', 0.65)
        self.declare_parameter('lookahead_m', 1.2)
        self.declare_parameter('front_arc_deg', 60.0)
        self.declare_parameter('costmap_width_m', 4.0)
        self.declare_parameter('costmap_height_m', 4.0)
        self.declare_parameter('costmap_res_m', 0.10)
        self.declare_parameter('inflation_radius_m', 0.30)

        gp = self.get_parameter
        self._auto_start = bool(gp('auto_start').value)
        self._control_rate = float(gp('control_rate_hz').value)
        self._target = Waypoint(float(gp('target_x').value),
                                float(gp('target_y').value))
        self._path_y = float(gp('path_y').value)
        self._target_heading = float(gp('target_heading_rad').value)
        self._reflex_stop_d = float(gp('reflex_stop_distance_m').value)
        self._max_angular = float(gp('max_angular_rps').value)
        self._kp_yaw = float(gp('kp_yaw').value)

        self._costmap_cfg = CostmapConfig(
            width_m=float(gp('costmap_width_m').value),
            height_m=float(gp('costmap_height_m').value),
            resolution_m=float(gp('costmap_res_m').value),
            inflation_radius_m=float(gp('inflation_radius_m').value),
        )
        self._planner_cfg = PlannerConfig(
            lookahead_m=float(gp('lookahead_m').value),
            safety_cost_threshold=float(gp('safety_cost_threshold').value),
            default_speed_mps=float(gp('default_speed_mps').value),
            slow_speed_mps=float(gp('slow_speed_mps').value),
        )

        # ── Runtime state ────────────────────────────────────────────
        self._started = self._auto_start
        self._last_scan: Optional[LaserScan] = None
        self._current_pose: Optional[Pose2D] = None
        self._current_yaw: float = 0.0
        self._detections: List[Detection] = []
        self._camera_active_class: str = "none"
        self._last_state: str = "IDLE"
        self._last_plan: Optional[LocalPlan] = None
        self._reflex_active: bool = False

        # ── QoS + Subs ────────────────────────────────────────────────
        qos_sensor = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_sensor)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        if HAS_VISION_MSGS:
            self.create_subscription(
                Detection3DArray, '/detected_objects',
                self._on_detection, 10)
            self.get_logger().info(
                "Detection3DArray subscriber active on /detected_objects")
        else:
            self.get_logger().warn(
                "vision_msgs unavailable — detection input disabled")

        # ── Pubs ──────────────────────────────────────────────────────
        self._pub_cmd = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self._pub_state_legacy = self.create_publisher(
            String, '/avoider_state', 10)
        self._pub_state_short = self.create_publisher(
            String, '/avoider/state', 10)
        self._pub_debug = self.create_publisher(String, '/avoider/debug', 10)

        # ── Servisler ────────────────────────────────────────────────
        self.create_service(Trigger, '/avoider/start', self._on_start_srv)
        self.create_service(Trigger, '/avoider/stop', self._on_stop_srv)

        # ── Tick ──────────────────────────────────────────────────────
        period = 1.0 / max(self._control_rate, 1.0)
        self.create_timer(period, self._tick)

        self.get_logger().info(
            f"DynamicLocalPlanner ready  auto_start={self._auto_start}  "
            f"target=({self._target.x:.1f},{self._target.y:.1f})  "
            f"speed={self._planner_cfg.default_speed_mps}m/s  "
            f"reflex={self._reflex_stop_d}m")

    # ──────────────────────────────────────────────────────────────────
    # Subs callbacks
    # ──────────────────────────────────────────────────────────────────
    def _on_scan(self, msg: LaserScan):
        self._last_scan = msg

    def _on_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self._current_yaw = _yaw_from_quaternion(q.x, q.y, q.z, q.w)
        p = msg.pose.pose.position
        self._current_pose = Pose2D(p.x, p.y, self._current_yaw)

    def _on_detection(self, msg):
        dets: List[Detection] = []
        active_class = "none"
        if not msg.detections:
            self._detections = []
            self._camera_active_class = "none"
            return
        min_d = float('inf')
        for det in msg.detections:
            try:
                p = det.bbox.center.position
                x = float(p.x)
                y = float(p.y)
            except (AttributeError, TypeError, ValueError):
                continue
            if not (math.isfinite(x) and math.isfinite(y)):
                continue
            try:
                cls = str(det.results[0].hypothesis.class_id) \
                    if det.results else "unknown"
            except (AttributeError, IndexError):
                cls = "unknown"
            dets.append(Detection(class_id=cls, x=x, y=y, confidence=1.0))
            d = math.hypot(x, y)
            if d < min_d:
                min_d = d
                active_class = cls
        self._detections = dets
        self._camera_active_class = active_class

    # ──────────────────────────────────────────────────────────────────
    # Servisler
    # ──────────────────────────────────────────────────────────────────
    def _on_start_srv(self, request, response):
        if self._started:
            response.success = True
            response.message = "already started"
            return response
        self._started = True
        self.get_logger().info("/avoider/start → planner ARMED")
        response.success = True
        response.message = "started"
        return response

    def _on_stop_srv(self, request, response):
        self._started = False
        self._pub_cmd.publish(Twist())
        self.get_logger().info("/avoider/stop → planner DISARMED")
        response.success = True
        response.message = "stopped"
        return response

    # ──────────────────────────────────────────────────────────────────
    # Tick — pipeline
    # ──────────────────────────────────────────────────────────────────
    def _tick(self):
        if not self._started:
            self._publish_idle()
            return
        if self._last_scan is None or self._current_pose is None:
            self._publish_idle(state="WAITING_SENSORS")
            return

        scan = self._last_scan
        pose = self._current_pose

        # 1) Costmap (robot frame)
        cm = build_costmap_from_scan(
            scan.ranges, scan.angle_min, scan.angle_increment,
            self._costmap_cfg,
        )
        # 2) Detection overlay (robot frame — sim_detection bbox doğrudan x ileri y sol)
        overlay_detections(cm, self._detections,
                           semantic_weights=_DEFAULT_SEMANTIC_WEIGHTS)
        # 3) Summary
        summary = summarize_costmap(cm)

        # 4) Behavior decision
        decision = select_behavior(
            detections=self._detections,
            costmap_summary=summary,
        )

        # 5) Plan
        plan = plan_local_waypoint(
            pose=pose, target_waypoint=self._target,
            costmap=cm, behavior_decision=decision,
            config=self._planner_cfg,
        )
        self._last_plan = plan

        # 6) Controller — basit go-to-waypoint
        cmd = Twist()
        if plan.success and plan.local_waypoint is not None:
            lwp = plan.local_waypoint
            dx = lwp.x - pose.x
            dy = lwp.y - pose.y
            target_angle = math.atan2(dy, dx)
            yaw_err = _wrap_pi(target_angle - pose.yaw)
            angular_z = _clip(self._kp_yaw * yaw_err,
                              -self._max_angular, self._max_angular)
            # Yaw'a göre forward kıs
            linear_x = plan.speed_mps * max(0.2, 1.0 - min(abs(yaw_err), 1.0))
            cmd.linear.x = float(linear_x)
            cmd.angular.z = float(angular_z)
            phase = "BYPASSING" if abs(lwp.y - pose.y) > 0.1 else "DRIVING"
        else:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            phase = "HOLD"

        # 7) Reflex safety — son savunma katmanı
        front_min = summary.get('min_obs_dist', -1.0)
        reflex = False
        if isinstance(front_min, (int, float)) and \
                front_min >= 0.0 and front_min < self._reflex_stop_d:
            reflex = True
            cmd.linear.x = 0.0
            # Boş tarafa kaç
            gaps = find_free_lateral_gaps(cm, lookahead_x=0.6,
                                          corridor_width=0.3)
            if gaps:
                # En geniş gap merkezi
                widest = max(gaps, key=lambda g: g[1] - g[0])
                mid_y = 0.5 * (widest[0] + widest[1])
                cmd.angular.z = _clip(2.0 * mid_y, -self._max_angular,
                                       self._max_angular)
            else:
                cmd.angular.z = self._max_angular * 0.5  # default sola dön
            phase = "REFLEX_STOP"
        self._reflex_active = reflex

        # 8) Publish
        self._pub_cmd.publish(cmd)
        self._last_state = phase
        self._publish_telemetry(
            phase=phase, summary=summary, decision=decision,
            plan=plan, cmd=cmd, scan=scan,
        )

    # ──────────────────────────────────────────────────────────────────
    # Telemetry
    # ──────────────────────────────────────────────────────────────────
    def _publish_idle(self, state: str = "IDLE"):
        self._pub_cmd.publish(Twist())
        # Sade state
        ss = String(); ss.data = state
        self._pub_state_short.publish(ss)
        # Legacy
        sj = String()
        sj.data = json.dumps({
            "phase": state, "started": self._started,
            "distance_clear_m": 0.0, "goal_heading": 0.0,
            "current_yaw": _safe(self._current_yaw, 0.0),
            "yaw_err": 0.0, "avoid_direction": 0,
            "lidar_min_obs": -1.0, "camera_min_obs": -1.0,
            "hazard_action": "CLEAR",
            "reason": "awaiting /avoider/start" if not self._started
                       else "awaiting sensors",
        }, allow_nan=False)
        self._pub_state_legacy.publish(sj)
        # Debug
        dbg = String()
        dbg.data = json.dumps({
            "state": state, "started": self._started,
            "front_min": -1.0, "left_min": -1.0, "right_min": -1.0,
            "chosen_side": "none",
            "obstacle_detected": False, "camera_detected": False,
            "active_class": self._camera_active_class,
            "current_yaw": _safe(self._current_yaw, 0.0),
            "target_yaw": _safe(self._target_heading, 0.0),
            "yaw_error": 0.0,
            "cmd_linear": 0.0, "cmd_angular": 0.0,
            # Dynamic ek alanlar
            "planner_mode": "idle",
            "behavior_mode": "drive",
            "local_waypoint_x": -1.0, "local_waypoint_y": 0.0,
            "plan_success": False, "plan_reason": "idle",
            "reflex_active": False,
            "target_x": _safe(self._target.x, 0.0),
            "target_y": _safe(self._target.y, 0.0),
        }, allow_nan=False)
        self._pub_debug.publish(dbg)

    def _publish_telemetry(self, phase: str, summary: dict,
                            decision: BehaviorDecision, plan: LocalPlan,
                            cmd: Twist, scan: LaserScan):
        # front_min from scan (raw lidar) — telemetri kararlılığı
        ranges = scan.ranges
        amin = scan.angle_min
        ainc = scan.angle_increment
        half_arc = math.radians(self.get_parameter('front_arc_deg').value) / 2.0
        front_vals, left_vals, right_vals = [], [], []
        for i, r in enumerate(ranges):
            try:
                rf = float(r)
            except (TypeError, ValueError):
                continue
            if rf <= 0 or not math.isfinite(rf):
                continue
            ang = _wrap_pi(amin + i * ainc)
            if abs(ang) > half_arc:
                continue
            front_vals.append(rf)
            (left_vals if ang >= 0 else right_vals).append(rf)
        f_min = min(front_vals) if front_vals else float('inf')
        l_min = min(left_vals) if left_vals else float('inf')
        r_min = min(right_vals) if right_vals else float('inf')

        # chosen_side
        if plan.success and plan.local_waypoint is not None:
            dy = plan.local_waypoint.y - self._current_pose.y
            chosen_side = "left" if dy > 0.05 else \
                          ("right" if dy < -0.05 else "none")
        else:
            chosen_side = "none"

        cam_detected = bool(self._detections)
        target_world_dx = self._target.x - self._current_pose.x
        target_world_dy = self._target.y - self._current_pose.y
        target_yaw = math.atan2(target_world_dy, target_world_dx) \
            if math.hypot(target_world_dx, target_world_dy) > 1e-6 else 0.0
        yaw_err = _wrap_pi(target_yaw - self._current_yaw)

        lwp_x = plan.local_waypoint.x if plan.local_waypoint else -1.0
        lwp_y = plan.local_waypoint.y if plan.local_waypoint else 0.0

        # /avoider/state sade
        ss = String(); ss.data = phase
        self._pub_state_short.publish(ss)

        # /avoider_state legacy
        sj = String()
        sj.data = json.dumps({
            "phase": phase, "started": True,
            "distance_clear_m": _safe(target_world_dx, 0.0),
            "goal_heading": _safe(target_yaw, 0.0),
            "current_yaw": _safe(self._current_yaw, 0.0),
            "yaw_err": _safe(yaw_err, 0.0),
            "avoid_direction": (1 if chosen_side == 'left'
                                else -1 if chosen_side == 'right' else 0),
            "lidar_min_obs": _safe(f_min),
            "camera_min_obs": -1.0,
            "hazard_action": "CLEAR",
            "reason": plan.reason or decision.reason,
        }, allow_nan=False)
        self._pub_state_legacy.publish(sj)

        # /avoider/debug zengin
        dbg = String()
        dbg.data = json.dumps({
            "state": phase, "started": True,
            "front_min": _safe(f_min),
            "left_min": _safe(l_min),
            "right_min": _safe(r_min),
            "chosen_side": chosen_side,
            "obstacle_detected": bool(summary.get('front_blocked', False)),
            "camera_detected": cam_detected,
            "active_class": self._camera_active_class,
            "current_yaw": _safe(self._current_yaw, 0.0),
            "target_yaw": _safe(target_yaw, 0.0),
            "yaw_error": _safe(yaw_err, 0.0),
            "cmd_linear": _safe(cmd.linear.x, 0.0),
            "cmd_angular": _safe(cmd.angular.z, 0.0),
            # Dynamic ek alanlar
            "planner_mode": plan.mode,
            "behavior_mode": decision.mode.value,
            "local_waypoint_x": _safe(lwp_x),
            "local_waypoint_y": _safe(lwp_y, 0.0),
            "plan_success": bool(plan.success),
            "plan_reason": str(plan.reason)[:120],
            "reflex_active": bool(self._reflex_active),
            "target_x": _safe(self._target.x, 0.0),
            "target_y": _safe(self._target.y, 0.0),
        }, allow_nan=False)
        self._pub_debug.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicLocalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
