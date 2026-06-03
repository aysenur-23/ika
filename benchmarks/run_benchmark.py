"""IKA Tez Benchmark Runner.

Her senaryo × mod × trial icin:
  1. sim_full launch et (asgari hazirlik beklemesiyle)
  2. Yorunge kaydet (/odometry/filtered)
  3. Goal gonder (nav2 modunda) veya avoider'i baslat (avoider modunda)
  4. Pass kriterleri / timeout'a kadar bekle
  5. Ham metrikleri CSV'ye yaz, yorungeyi npz'e kaydet
  6. Sim'i kapat, sonraki

Cikti yapisi:
  benchmarks/results/raw_runs.csv          - her run icin 1 satir
  benchmarks/results/trajectories/<id>.npz - (T, 3) ndarray (t, x, y)
  benchmarks/results/plots/                - tablo + plot ciktilari

Kullanim:
  python3 benchmarks/run_benchmark.py --scenarios all --trials 5
  python3 benchmarks/run_benchmark.py --scenarios s2_ramp_climb --modes avoider
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Optional, Sequence

try:
    import yaml
    import numpy as np
except ImportError as e:
    print(f"Eksik bagimlilik: {e}. pip install pyyaml numpy", file=sys.stderr)
    sys.exit(1)

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped
    from std_msgs.msg import String
    ROS_AVAILABLE = True
except ImportError:
    ROS_AVAILABLE = False


BENCHMARK_DIR = Path(__file__).resolve().parent
WORKSPACE_DIR = BENCHMARK_DIR.parent
RESULTS_DIR = BENCHMARK_DIR / "results"


@dataclass
class RunRecord:
    scenario_id: str
    mode_id: str
    trial_idx: int
    success: bool
    duration_s: float
    path_length_m: float
    min_clearance_m: float
    mean_abs_curvature: float
    avg_speed_mps: float
    num_recoveries: int
    error_code: int
    notes: str


def load_scenarios(yaml_path: Path) -> dict:
    with open(yaml_path) as f:
        return yaml.safe_load(f)


def _yaw_from_quat(x, y, z, w) -> float:
    return math.atan2(2.0 * (w * z + x * y),
                      1.0 - 2.0 * (y * y + z * z))


class RunRecorder(Node):
    """Bir trial boyunca /odom + /avoider_state + /goal_pose dinler."""

    def __init__(self, scenario_id: str, mode_id: str):
        super().__init__(f'benchmark_recorder')
        self.scenario_id = scenario_id
        self.mode_id = mode_id
        self.trajectory: list[tuple[float, float, float]] = []  # (t, x, y)
        self.avoider_phases: list[tuple[float, str]] = []       # (t, phase)
        self.start_time: Optional[float] = None
        self.goal_reached = False
        self.num_recoveries = 0
        self.last_error_code = 0
        self.create_subscription(Odometry, '/odometry/filtered', self._odom_cb, 10)
        self.create_subscription(String, '/avoider_state', self._avoider_cb, 10)

    def _odom_cb(self, msg: Odometry):
        if self.start_time is None:
            self.start_time = time.time()
        t = time.time() - self.start_time
        p = msg.pose.pose.position
        self.trajectory.append((t, p.x, p.y))

    def _avoider_cb(self, msg: String):
        try:
            payload = json.loads(msg.data)
            phase = payload.get('phase', '')
            if self.start_time is not None:
                t = time.time() - self.start_time
                self.avoider_phases.append((t, phase))
                if phase == 'DONE':
                    self.goal_reached = True
        except (ValueError, TypeError):
            pass


def path_length(points: Sequence[tuple]) -> float:
    total = 0.0
    for (_, x0, y0), (_, x1, y1) in zip(points, points[1:]):
        total += math.hypot(x1 - x0, y1 - y0)
    return total


def min_clearance(points, obstacles) -> float:
    if not obstacles or not points:
        return float('inf')
    best = float('inf')
    for ox, oy in obstacles:
        for _, x, y in points:
            d = math.hypot(x - ox, y - oy)
            best = min(best, d)
    return best


def mean_abs_curvature(points) -> float:
    if len(points) < 3:
        return 0.0
    headings = []
    seg_lens = []
    for (_, x0, y0), (_, x1, y1) in zip(points, points[1:]):
        dx, dy = x1 - x0, y1 - y0
        d = math.hypot(dx, dy)
        if d < 1e-4:
            continue
        headings.append(math.atan2(dy, dx))
        seg_lens.append(d)
    if len(headings) < 2:
        return 0.0
    total_turn = 0.0
    total_len = 0.0
    for i in range(1, len(headings)):
        dtheta = math.atan2(math.sin(headings[i] - headings[i-1]),
                            math.cos(headings[i] - headings[i-1]))
        total_turn += abs(dtheta)
        total_len += seg_lens[i]
    if total_len < 1e-6:
        return 0.0
    return total_turn / total_len


def launch_sim(mode_args: dict) -> subprocess.Popen:
    """sim_full.launch.py'i belirtilen arg'larla baslatir, popen doner."""
    args = ['ros2', 'launch', 'ika_bringup', 'sim_full.launch.py']
    for k, v in mode_args.items():
        args.append(f'{k}:={v}')
    print(f"[bench] LAUNCH: {' '.join(args)}")
    proc = subprocess.Popen(
        args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        preexec_fn=os.setsid)
    return proc


def stop_sim(proc: subprocess.Popen):
    """Sim'i kapat (SIGINT then SIGKILL after grace)."""
    try:
        os.killpg(os.getpgid(proc.pid), signal.SIGINT)
        try:
            proc.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            proc.wait(timeout=5)
    except ProcessLookupError:
        pass
    # Stray gz sim temizleme
    subprocess.run(['pkill', '-9', '-f', 'gz sim server'], check=False)
    subprocess.run(['pkill', '-9', '-f', 'gz sim gui'], check=False)


def send_nav2_goal(goal_x: float, goal_y: float):
    """Nav2 modunda /goal_pose'a hedef yayinla."""
    cmd = [
        'ros2', 'topic', 'pub', '--once', '/goal_pose',
        'geometry_msgs/PoseStamped',
        f'{{header: {{frame_id: "map"}}, pose: {{position: {{x: {goal_x}, y: {goal_y}, z: 0.0}}, orientation: {{w: 1.0}}}}}}',
    ]
    subprocess.run(cmd, check=False, timeout=10)


def run_one(scenario: dict, mode: dict, trial_idx: int) -> RunRecord:
    if not ROS_AVAILABLE:
        raise RuntimeError("ROS 2 Python paketleri yok — workspace'i source'la")

    sid = scenario['id']
    mid = mode['id']
    timeout_s = scenario['pass_criteria'].get('max_duration_s', 60)
    obstacles = scenario.get('obstacles_for_clearance', [])
    goal_x = scenario['goal']['x']
    goal_y = scenario['goal']['y']
    is_nav2 = mode['launch_args'].get('autonomous_mode') == 'nav2'

    print(f"\n=== [{sid}] mode={mid} trial={trial_idx+1} ===")

    # 1. Sim'i baslat
    proc = launch_sim(mode['launch_args'])

    # 2. Bringup bekle (45 sn)
    print("[bench] bringup beklemesi (45 sn)...")
    time.sleep(45)

    # 3. ROS recorder baslat
    rclpy.init(args=[])
    recorder = RunRecorder(sid, mid)

    # 4. Hedef gonder (nav2 modunda)
    if is_nav2:
        time.sleep(2)
        send_nav2_goal(goal_x, goal_y)

    # 5. Yorunge topla + timeout bekle
    start = time.time()
    last_status_print = start
    while time.time() - start < timeout_s:
        rclpy.spin_once(recorder, timeout_sec=0.1)
        # Periyodik durum
        if time.time() - last_status_print > 10:
            elapsed = time.time() - start
            print(f"[bench] t={elapsed:.0f}s  points={len(recorder.trajectory)}")
            last_status_print = time.time()
        # Avoider DONE state bitti say
        if recorder.goal_reached:
            print("[bench] avoider DONE — bitti")
            break
        # Nav2 modunda goal'e ulasti mi diye konum kontrolu
        if recorder.trajectory and is_nav2:
            _, lx, ly = recorder.trajectory[-1]
            dist_to_goal = math.hypot(lx - goal_x, ly - goal_y)
            if dist_to_goal < 0.30:
                print(f"[bench] goal'e ulasildi (d={dist_to_goal:.2f})")
                recorder.goal_reached = True
                break

    duration = time.time() - start

    # 6. Metrikleri hesapla
    pts = recorder.trajectory
    plen = path_length(pts)
    success = bool(recorder.goal_reached and len(pts) > 0)
    clearance = min_clearance(pts, obstacles)
    curv = mean_abs_curvature(pts)
    avg_speed = (plen / duration) if duration > 1e-6 else 0.0

    record = RunRecord(
        scenario_id=sid, mode_id=mid, trial_idx=trial_idx,
        success=success, duration_s=round(duration, 2),
        path_length_m=round(plen, 3),
        min_clearance_m=(round(clearance, 3)
                         if not math.isinf(clearance) else float('inf')),
        mean_abs_curvature=round(curv, 4),
        avg_speed_mps=round(avg_speed, 3),
        num_recoveries=recorder.num_recoveries,
        error_code=recorder.last_error_code,
        notes="",
    )

    # 7. Yorungeyi npz olarak kaydet
    traj_dir = RESULTS_DIR / 'trajectories'
    traj_dir.mkdir(parents=True, exist_ok=True)
    arr = np.array(pts, dtype=float) if pts else np.zeros((0, 3))
    np.savez(traj_dir / f'{sid}_{mid}_t{trial_idx}.npz',
             trajectory=arr,
             goal=np.array([goal_x, goal_y]),
             obstacles=np.array(obstacles) if obstacles else np.zeros((0, 2)),
             metadata=json.dumps({
                 'scenario_id': sid, 'mode_id': mid, 'trial': trial_idx,
                 'duration_s': duration, 'success': success,
             }))

    # 8. Cleanup
    recorder.destroy_node()
    rclpy.shutdown()
    stop_sim(proc)
    print(f"[bench] sim kapandi, cool-down 5 sn")
    time.sleep(5)

    return record


def write_csv(records: list[RunRecord], path: Path):
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(asdict(records[0]).keys()) if records else [])
        if records:
            w.writeheader()
            for r in records:
                w.writerow(asdict(r))
    print(f"[bench] {len(records)} run -> {path}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--scenarios', type=str, default='all',
                   help="virgülle ayrilmis ID veya 'all'")
    p.add_argument('--modes', type=str, default='all',
                   help="virgülle ayrilmis mode_id veya 'all'")
    p.add_argument('--trials', type=int, default=None,
                   help="trial sayisi (default scenarios.yaml'dan)")
    p.add_argument('--config', type=Path,
                   default=BENCHMARK_DIR / 'scenarios.yaml')
    args = p.parse_args()

    cfg = load_scenarios(args.config)
    all_scenarios = cfg['scenarios']
    all_modes = cfg['modes']
    trials = args.trials or cfg['trials_per_combo']

    if args.scenarios != 'all':
        ids = args.scenarios.split(',')
        all_scenarios = [s for s in all_scenarios if s['id'] in ids]
    if args.modes != 'all':
        ids = args.modes.split(',')
        all_modes = [m for m in all_modes if m['id'] in ids]

    print(f"[bench] {len(all_scenarios)} senaryo × {len(all_modes)} mod × "
          f"{trials} trial = {len(all_scenarios)*len(all_modes)*trials} run")

    records: list[RunRecord] = []
    for scenario in all_scenarios:
        for mode in all_modes:
            for trial in range(trials):
                try:
                    rec = run_one(scenario, mode, trial)
                    records.append(rec)
                    write_csv(records, RESULTS_DIR / cfg['output']['raw_csv'])
                except Exception as e:
                    print(f"[bench] HATA {scenario['id']}/{mode['id']}/{trial}: {e}")

    print(f"\n[bench] tamamlandi: {len(records)} run kaydedildi")
    print(f"[bench] sonuc: {RESULTS_DIR / cfg['output']['raw_csv']}")
    print(f"[bench] sonraki: python3 benchmarks/render_tables.py")


if __name__ == '__main__':
    main()
