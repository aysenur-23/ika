#!/usr/bin/env python3
"""Faz 3.0 — /plan + /local_plan topic snapshot.

Sim çalışıyorsa: goal yolla, /plan'ın ilk mesajını yakala, analiz et,
yazdır.

Kullanım:
  python3 capture_plan.py --out /tmp/plan_snapshot.json

Hipotez ölçümü:
  - Planner çıkışı düz mü, kıvrımlı mı?
  - Engelin (1.5, 0) yan tarafından mı geçiyor?
  - Kaç poz, hangi y aralığı?
"""
import argparse
import json
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import NavigateToPose


OBSTACLE_XY = (1.5, 0.0)
GOAL_XY = (3.0, 0.0)


class PlanCapture(Node):
    def __init__(self):
        super().__init__('plan_capture')
        self.plan_poses = None
        self.local_plan_poses = None
        self.map_received = False
        self.local_cm_received = False
        self.global_cm_received = False

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, '/map',
                                 lambda _: setattr(self, 'map_received', True),
                                 latched)
        self.create_subscription(OccupancyGrid, '/local_costmap/costmap',
                                 lambda _: setattr(self, 'local_cm_received', True),
                                 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda _: setattr(self, 'global_cm_received', True),
                                 10)
        self.create_subscription(Path, '/plan', self._plan_cb, 10)
        self.create_subscription(Path, '/local_plan', self._local_plan_cb, 10)

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

    def _plan_cb(self, msg: Path):
        # Her /plan mesajini guncelle: son alinan plan kullanilir (planner
        # costmap update'i ile beraber yeniler).
        self.plan_count = getattr(self, 'plan_count', 0) + 1
        self.plan_poses = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]

    def _local_plan_cb(self, msg: Path):
        self.local_plan_count = getattr(self, 'local_plan_count', 0) + 1
        self.local_plan_poses = [
            (p.pose.position.x, p.pose.position.y) for p in msg.poses
        ]

    def wait_ready(self, timeout=60.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.2)
            if (self.map_received and self.local_cm_received
                    and self.global_cm_received):
                break
        else:
            self.get_logger().error('Probe timeout (map/costmap topic)')
            return False
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Nav2 server timeout')
            return False
        # extra settle
        end = time.time() + 3.0
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return True

    def send_goal(self):
        g = NavigateToPose.Goal()
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = GOAL_XY[0]
        p.pose.position.y = GOAL_XY[1]
        p.pose.orientation.w = 1.0
        g.pose = p
        return self.nav_client.send_goal_async(g)

    def capture(self, capture_time=8.0):
        # 8 saniye boyunca /plan + /local_plan dinle
        end = time.time() + capture_time
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)


def analyze_plan(poses):
    if not poses:
        return None
    xs = [p[0] for p in poses]
    ys = [p[1] for p in poses]
    n = len(poses)
    arc_len = sum(math.hypot(xs[i+1]-xs[i], ys[i+1]-ys[i]) for i in range(n-1))
    straight_len = math.hypot(xs[-1]-xs[0], ys[-1]-ys[0])
    # path engelden geçiyor mu?
    ox, oy = OBSTACLE_XY
    obs_clearance = min(math.hypot(x-ox, y-oy) for x, y in poses)
    # max yan sapma
    y_min, y_max = min(ys), max(ys)
    # plan engelin etrafından mı dolanıyor? (engelin x'i geçen poz var mı,
    # ve o pozlarda |y| > 0.5 mi?)
    bypass_y = max((abs(y) for x, y in poses if abs(x - ox) < 0.3), default=0.0)
    return {
        'n_poses': n,
        'start': poses[0],
        'end': poses[-1],
        'arc_length_m': round(arc_len, 3),
        'straight_dist_m': round(straight_len, 3),
        'arc_over_straight': round(arc_len / max(straight_len, 1e-6), 3),
        'min_obs_clearance_m': round(obs_clearance, 3),
        'y_range_m': [round(y_min, 3), round(y_max, 3)],
        'bypass_y_near_obstacle': round(bypass_y, 3),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='/tmp/plan_snapshot.json')
    ap.add_argument('--capture-s', type=float, default=8.0)
    args = ap.parse_args()

    rclpy.init()
    node = PlanCapture()

    if not node.wait_ready():
        print('FAIL: stack not ready', file=sys.stderr)
        sys.exit(1)
    print('Stack hazır, goal yollanıyor...', file=sys.stderr)
    fut = node.send_goal()
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    if not fut.done():
        print('FAIL: goal send timeout', file=sys.stderr)
        sys.exit(1)
    print(f'/plan + /local_plan {args.capture_s}s yakalanıyor...', file=sys.stderr)
    node.capture(args.capture_s)

    plan_analysis = analyze_plan(node.plan_poses)
    local_plan_analysis = analyze_plan(node.local_plan_poses)

    result = {
        'global_plan': plan_analysis,
        'local_plan': local_plan_analysis,
        'plan_msg_count': getattr(node, 'plan_count', 0),
        'local_plan_msg_count': getattr(node, 'local_plan_count', 0),
        'plan_poses_xy': node.plan_poses,
        'local_plan_poses_xy': node.local_plan_poses,
    }
    with open(args.out, 'w') as f:
        json.dump(result, f, indent=2)
    print(f'Kaydedildi: {args.out}', file=sys.stderr)

    # Stdout'a özet (CSV-friendly)
    print(json.dumps({
        'global_plan': plan_analysis,
        'local_plan': local_plan_analysis,
    }, indent=2))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
