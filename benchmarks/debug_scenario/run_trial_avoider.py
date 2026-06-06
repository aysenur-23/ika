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
DEFAULT_PASS_X = 2.5                # bu x'i geçince "engel atlatıldı" (engel 1.5 + 1.0 m geçiş)

# TASK-2: strict PASS eşikleri
DEFAULT_SAFETY_MARGIN = 0.30
DEFAULT_ALLOWED_FINISH_ERR = 0.75
DEFAULT_ALLOWED_CORRIDOR = 2.00
DEFAULT_STUCK_THRESHOLD = 5.0
STUCK_SPEED_M = 0.05  # bu altında "hareketsiz" sayılır (m / sample window)


class AvoiderTrialMonitor(Node):
    def __init__(self,
                 obstacles=None,
                 collision_threshold: float = DEFAULT_COLLISION_THRESHOLD,
                 pass_x: float = DEFAULT_PASS_X,
                 safety_margin: float = DEFAULT_SAFETY_MARGIN,
                 allowed_finish_err: float = DEFAULT_ALLOWED_FINISH_ERR,
                 allowed_corridor: float = DEFAULT_ALLOWED_CORRIDOR,
                 stuck_threshold: float = DEFAULT_STUCK_THRESHOLD):
        super().__init__('avoider_trial_monitor')
        self.obstacles = obstacles or DEFAULT_OBSTACLES
        self.collision_threshold = collision_threshold
        self.pass_x = pass_x
        self.safety_margin = safety_margin
        self.allowed_finish_err = allowed_finish_err
        self.allowed_corridor = allowed_corridor
        self.stuck_threshold = stuck_threshold

        # State
        self.odom_xy = None
        self.start_xy = None
        self.min_obs_dist = float('inf')
        self.collision_triggered = False
        self.avoider_phase = "UNKNOWN"
        self.distance_clear_m = 0.0
        self.t_start = time.time()

        # TASK-2 telemetri
        self.max_y_deviation = 0.0
        self._state_transitions = 0
        self._last_state_seen = None
        # State transition kaynağı önceliği: "debug" > "short" > "legacy".
        # Yüksek öncelikli kaynak bir kez görüldükten sonra düşük öncelikliler
        # state transition sayacına KATKI yapmaz (çift sayım önlenir).
        self._state_source: str = "none"
        self._SOURCE_PRIORITY = {"none": 0, "legacy": 1, "short": 2, "debug": 3}
        self.stuck_time = 0.0           # max kesintisiz hareketsizlik süresi
        self._stuck_run_start = None    # şu anki hareketsiz aralığın başlangıcı
        # cmd_vel_oscillation_score: SADECE cmd_angular işaret değişim oranı.
        # (cmd_linear dahil değil.) Pencere içindeki sign-flip / (N-1).
        self._ang_z_history: list = []
        self._ang_z_window = 40         # ~4 sn @ 10Hz
        self.cmd_vel_oscillation_score = 0.0
        self._have_debug_topic = False

        # Subscriptions
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        scan_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(LaserScan, '/scan', self._scan_cb, scan_qos)
        self.create_subscription(String, '/avoider_state', self._avoider_state_cb, 10)
        # TASK-2: yeni telemetri topic'leri — yoksa graceful fallback
        self.create_subscription(String, '/avoider/state', self._avoider_state_short_cb, 10)
        self.create_subscription(String, '/avoider/debug', self._avoider_debug_cb, 10)

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.odom_xy = (p.x, p.y)
        if self.start_xy is None:
            self.start_xy = (p.x, p.y)
        # max y sapması (start_y referansına göre)
        if self.start_xy is not None:
            dy = abs(p.y - self.start_xy[1])
            if dy > self.max_y_deviation:
                self.max_y_deviation = dy

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
            new_phase = str(payload.get('phase', 'UNKNOWN'))
            self._note_state(new_phase, source="legacy")
            self.avoider_phase = new_phase
            self.distance_clear_m = float(payload.get('distance_clear_m', 0.0))
        except (ValueError, TypeError):
            pass

    def _avoider_state_short_cb(self, msg: String):
        # /avoider/state (sade string). State transition fallback kaynağı.
        phase = (msg.data or "").strip()
        if phase:
            self._note_state(phase, source="short")

    def _avoider_debug_cb(self, msg: String):
        # /avoider/debug — cmd_angular örnekle, state transition güncelle.
        self._have_debug_topic = True
        try:
            d = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        phase = str(d.get('state', '')) or None
        if phase:
            self._note_state(phase, source="debug")
        try:
            ang = float(d.get('cmd_angular', 0.0))
        except (TypeError, ValueError):
            ang = 0.0
        self._ang_z_history.append(ang)
        if len(self._ang_z_history) > self._ang_z_window:
            self._ang_z_history.pop(0)
        # Oscillation skoru: işaret değişiklik sayısı / pencere
        h = self._ang_z_history
        if len(h) >= 2:
            flips = 0
            for i in range(1, len(h)):
                if (h[i - 1] > 1e-3 and h[i] < -1e-3) or \
                   (h[i - 1] < -1e-3 and h[i] > 1e-3):
                    flips += 1
            self.cmd_vel_oscillation_score = flips / max(len(h) - 1, 1)

    def _note_state(self, phase: str, source: str = "legacy"):
        """State transition sayımı — kaynak önceliğine göre.

        Öncelik: debug > short > legacy. Daha yüksek öncelikli kaynak
        bir kez görüldüyse, düşük öncelikli kaynak GÖZ ARDI edilir
        (sadece avoider_phase için bilgi olarak kalır, sayaca eklenmez).
        """
        prio = self._SOURCE_PRIORITY.get(source, 0)
        active_prio = self._SOURCE_PRIORITY.get(self._state_source, 0)
        if prio < active_prio:
            return
        if prio > active_prio:
            # Yeni daha yüksek öncelikli kaynak devreye girdi: sayaç sıfırlanmaz
            # ama referans bu kaynağa göre yeniden hizalanır.
            self._state_source = source
            self._last_state_seen = phase
            return
        # Aynı kaynak — normal transition sayımı
        if self._last_state_seen is None:
            self._last_state_seen = phase
            return
        if phase != self._last_state_seen:
            self._state_transitions += 1
            self._last_state_seen = phase

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
        """Avoider hareketini izle, sonuç döndür.

        Avoider 'mission mode'da (target_distance_m=10000) sürekli sürer;
        DONE'a varmaz. PASS kriteri: robot ENGELİ AŞTI (x >= pass_x) + ÇARPMADI.

        Bu, kullanıcının asıl sorusunun cevabı: "engeli geçince yoluna devam".
        """
        self.t_start = time.time()
        deadline = self.t_start + timeout_s
        last_pos_change_t = self.t_start
        last_pos = self.odom_xy

        while time.time() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

            # Çarpma kontrolü
            if self.collision_triggered:
                return self._mkresult('FAIL_COLL', 'cancelled_by_collision')

            # PASS koşulu: robot engeli aştı (x >= pass_x) VE çarpmadı
            if self.odom_xy is not None and self.odom_xy[0] >= self.pass_x:
                return self._mkresult('PASS', f'passed_x>={self.pass_x:.2f}')

            # Avoider DONE'a geldi mi? (target_distance_m sonlu ise)
            if self.avoider_phase == "DONE":
                if self.collision_triggered:
                    return self._mkresult('FAIL_COLL', 'done_but_collided')
                if self.odom_xy is not None and self.odom_xy[0] >= self.pass_x:
                    return self._mkresult('PASS', 'avoider_DONE_passed')
                # DONE ama engeli aşmadı (yana saplandı)
                return self._mkresult('FAIL_REROUTED', 'avoider_DONE_but_not_past_obstacle')

            # Stuck detection: 8 saniye boyunca < 5 cm hareket
            now = time.time()
            if last_pos is not None and self.odom_xy is not None:
                d = math.hypot(self.odom_xy[0] - last_pos[0],
                               self.odom_xy[1] - last_pos[1])
                if d > STUCK_SPEED_M:
                    last_pos = self.odom_xy
                    last_pos_change_t = now
                    self._stuck_run_start = None
                else:
                    if self._stuck_run_start is None:
                        self._stuck_run_start = last_pos_change_t
                    run = now - self._stuck_run_start
                    if run > self.stuck_time:
                        self.stuck_time = run
                    if (now - last_pos_change_t) > 8.0:
                        return self._mkresult('FAIL_STUCK',
                                              f'stuck_{int(now - last_pos_change_t)}s')
            elif self.odom_xy is not None:
                last_pos = self.odom_xy
                last_pos_change_t = now

        return self._mkresult('FAIL_TIMEOUT', 'timeout')

    def _mkresult(self, status: str, reason: str) -> dict:
        rx, ry = self.odom_xy if self.odom_xy else (float('nan'), float('nan'))
        duration = time.time() - self.t_start
        sy = self.start_xy[1] if self.start_xy else 0.0
        final_y_err = (ry - sy) if math.isfinite(ry) else float('nan')
        min_obs = self.min_obs_dist if math.isfinite(self.min_obs_dist) else -1.0
        finish_reached = bool(math.isfinite(rx) and rx >= self.pass_x)
        collision = bool(self.collision_triggered)
        # TASK-2 strict PASS
        pass_strict = (
            finish_reached
            and not collision
            and min_obs >= self.safety_margin
            and math.isfinite(final_y_err)
            and abs(final_y_err) <= self.allowed_finish_err
            and self.max_y_deviation <= self.allowed_corridor
            and self.stuck_time <= self.stuck_threshold
        )
        return {
            'status': status,
            'final_x': rx,
            'final_y': ry,
            'min_obs_dist': min_obs,
            'distance_clear_m': self.distance_clear_m,
            'avoider_phase': self.avoider_phase,
            'duration': duration,
            'reason': reason,
            # TASK-2 yeni alanlar
            'finish_reached': finish_reached,
            'collision': collision,
            'min_obstacle_distance': min_obs,
            'max_y_deviation': round(self.max_y_deviation, 3),
            'final_y_error': (round(final_y_err, 3)
                              if math.isfinite(final_y_err) else float('nan')),
            'state_transition_count': self._state_transitions,
            'stuck_time': round(self.stuck_time, 3),
            'cmd_vel_oscillation_score': round(self.cmd_vel_oscillation_score, 3),
            'trial_duration': round(duration, 3),
            'pass_strict': pass_strict,
            'have_debug_topic': self._have_debug_topic,
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
    # TASK-2 strict PASS eşikleri
    ap.add_argument('--safety-margin', type=float, default=DEFAULT_SAFETY_MARGIN)
    ap.add_argument('--allowed-finish-error', type=float,
                    default=DEFAULT_ALLOWED_FINISH_ERR)
    ap.add_argument('--allowed-corridor-width', type=float,
                    default=DEFAULT_ALLOWED_CORRIDOR)
    ap.add_argument('--stuck-threshold', type=float,
                    default=DEFAULT_STUCK_THRESHOLD)
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
        safety_margin=args.safety_margin,
        allowed_finish_err=args.allowed_finish_error,
        allowed_corridor=args.allowed_corridor_width,
        stuck_threshold=args.stuck_threshold,
    )

    if not node.wait_for_avoider_ready(timeout=args.ready_timeout):
        result = {
            'status': 'FAIL_NOT_READY', 'final_x': float('nan'),
            'final_y': float('nan'), 'min_obs_dist': -1.0,
            'distance_clear_m': 0.0, 'avoider_phase': 'UNKNOWN',
            'duration': 0.0, 'reason': 'avoider_node_not_ready',
            'finish_reached': False, 'collision': False,
            'min_obstacle_distance': -1.0, 'max_y_deviation': 0.0,
            'final_y_error': float('nan'), 'state_transition_count': 0,
            'stuck_time': 0.0, 'cmd_vel_oscillation_score': 0.0,
            'trial_duration': 0.0, 'pass_strict': False,
            'have_debug_topic': False,
        }
    else:
        result = node.run_trial(args.timeout)

    node.destroy_node()
    rclpy.shutdown()

    def _f(x, fmt=".3f"):
        try:
            xf = float(x)
        except (TypeError, ValueError):
            return ""
        if not math.isfinite(xf):
            return ""
        return format(xf, fmt)

    buf = StringIO()
    writer = csv.writer(buf)
    writer.writerow([
        # ── ESKİ ALANLAR (sıra korunur) ──────────────────────────────
        args.trial_id, result['status'],
        _f(result['final_x']), _f(result['final_y']),
        _f(result['min_obs_dist']), _f(result['distance_clear_m']),
        result['avoider_phase'], _f(result['duration'], '.2f'),
        result['reason'],
        # ── TASK-2 YENİ ALANLAR (append) ─────────────────────────────
        int(bool(result['finish_reached'])),
        int(bool(result['collision'])),
        _f(result['min_obstacle_distance']),
        _f(result['max_y_deviation']),
        _f(result['final_y_error']),
        int(result['state_transition_count']),
        _f(result['stuck_time']),
        _f(result['cmd_vel_oscillation_score']),
        _f(result['trial_duration'], '.2f'),
        int(bool(result['pass_strict'])),
    ])
    sys.stdout.write(buf.getvalue())
    sys.stdout.flush()
    # Exit kodu SADECE strict PASS'e göre. Eski 'status' CSV'de kalır ama
    # process success artık strict PASS'tir.
    ok = bool(result['pass_strict'])
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
