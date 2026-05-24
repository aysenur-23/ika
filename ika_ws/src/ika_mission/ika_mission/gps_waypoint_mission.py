"""IKA - GPS Waypoint Mission.

YAML dosyasindaki waypoint listesini sirayla Nav2'ye gonderir.
GPS koordinatlari icin navsat_transform_node calismali (UTM -> map donusumu).

Yayinlanan topic'ler:
  /mission_state (std_msgs/String, JSON) - guncel gorev durumu
  /mission_log   (std_msgs/String)        - insan-okunabilir log satirlari

Abone olunan topic'ler:
  /mission_cmd (std_msgs/String) - dis komutlar:
     "cancel"  -> gorevi iptal et, mevcut hedefi durdur
     "pause"   -> ileri yollama gecici durdur, mevcut hedef devam
     "resume"  -> pause sonrasi devam
     "skip"    -> mevcut waypoint'i atla, bir sonrakine gec
     "restart" -> ilk waypoint'ten basla

Argumanlar:
  --waypoints /path/to/mission.yaml   (varsayilan: paket icindeki test_mission.yaml)
  --frame map                          (waypoint'lerin frame'i)
  --max-retries 1                      (basarisiz waypoint icin tekrar deneme sayisi)

YAML formati:
  waypoints:
    - x: 2.0
      y: 0.5
      yaw: 0.0     # opsiyonel
      label: hedef_1
      tolerance: 0.25   # opsiyonel - xy_goal_tolerance override
"""
import argparse
import json
import math
import os
import sys
from typing import List, Optional

import yaml

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GPSWaypointMission(Node):

    STATE_IDLE = 'idle'
    STATE_RUNNING = 'running'
    STATE_PAUSED = 'paused'
    STATE_CANCELLED = 'cancelled'
    STATE_DONE = 'done'

    def __init__(self, waypoints_path: str, frame: str = 'map',
                 max_retries: int = 1):
        super().__init__('gps_waypoint_mission')
        self.frame = frame
        self.max_retries = int(max_retries)

        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._cmd_sub = self.create_subscription(
            String, '/mission_cmd', self._on_cmd, 10)
        self._state_pub = self.create_publisher(String, '/mission_state', 10)
        self._log_pub = self.create_publisher(String, '/mission_log', 10)

        self.waypoints: List[dict] = self._load(waypoints_path)
        self.idx = 0
        self.retries_left = self.max_retries
        self.state = self.STATE_IDLE
        self._current_goal_handle = None
        self._waypoints_path = waypoints_path

        self.create_timer(1.0, self._publish_state)
        self._log(f'{len(self.waypoints)} waypoint yuklendi: {waypoints_path}')

    # ---- IO -----------------------------------------------------------
    def _load(self, path: str) -> List[dict]:
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        return list(data.get('waypoints') or [])

    def _log(self, message: str):
        self.get_logger().info(message)
        msg = String()
        msg.data = message
        self._log_pub.publish(msg)

    def _publish_state(self):
        payload = {
            'state': self.state,
            'index': self.idx,
            'total': len(self.waypoints),
            'retries_left': self.retries_left,
            'waypoints_path': self._waypoints_path,
            'current_label': (
                self.waypoints[self.idx].get('label', f'wp_{self.idx + 1}')
                if 0 <= self.idx < len(self.waypoints) else None
            ),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._state_pub.publish(msg)

    # ---- public API ---------------------------------------------------
    def start(self):
        if not self.waypoints:
            self._log('Waypoint listesi bos - cikiliyor')
            self.state = self.STATE_DONE
            rclpy.shutdown()
            return
        if not self._nav.wait_for_server(timeout_sec=15.0):
            self._log('HATA: navigate_to_pose action server bulunamadi')
            self.state = self.STATE_DONE
            rclpy.shutdown()
            return
        self.state = self.STATE_RUNNING
        self._send_current()

    # ---- komut callback ----------------------------------------------
    def _on_cmd(self, msg: String):
        cmd = msg.data.strip().lower()
        self._log(f'Komut alindi: {cmd}')
        if cmd == 'cancel':
            self.state = self.STATE_CANCELLED
            self._cancel_current_goal()
        elif cmd == 'pause':
            if self.state == self.STATE_RUNNING:
                self.state = self.STATE_PAUSED
                self._cancel_current_goal()
        elif cmd == 'resume':
            if self.state == self.STATE_PAUSED:
                self.state = self.STATE_RUNNING
                self._send_current()
        elif cmd == 'skip':
            self._cancel_current_goal()
            self.idx += 1
            self.retries_left = self.max_retries
            if self.state == self.STATE_RUNNING:
                self._send_current()
        elif cmd == 'restart':
            self._cancel_current_goal()
            self.idx = 0
            self.retries_left = self.max_retries
            self.state = self.STATE_RUNNING
            self._send_current()
        else:
            self._log(f'Bilinmeyen komut: {cmd}')

    def _cancel_current_goal(self):
        if self._current_goal_handle is None:
            return
        handle = self._current_goal_handle
        self._current_goal_handle = None
        try:
            handle.cancel_goal_async()
        except Exception as exc:
            self._log(f'Iptal hatasi: {exc}')

    # ---- gorev ilerletme ----------------------------------------------
    def _send_current(self):
        if self.state != self.STATE_RUNNING:
            return
        if self.idx >= len(self.waypoints):
            self._log('Tum waypoint\'ler tamamlandi')
            self.state = self.STATE_DONE
            rclpy.shutdown()
            return

        wp = self.waypoints[self.idx]
        x = float(wp['x'])
        y = float(wp['y'])
        yaw = float(wp.get('yaw', 0.0))
        label = wp.get('label', f'wp_{self.idx + 1}')

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = self.frame
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quat(yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self._log(
            f'-> Waypoint {self.idx + 1}/{len(self.waypoints)} '
            f'[{label}] x={x:.2f} y={y:.2f} yaw={yaw:.2f} '
            f'retries_left={self.retries_left}')

        future = self._nav.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        handle = future.result()
        if handle is None or not handle.accepted:
            self._log('Hedef reddedildi - waypoint atlaniyor')
            self._advance(success=False)
            return
        self._current_goal_handle = handle
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._on_result)

    def _on_result(self, future):
        self._current_goal_handle = None
        try:
            result = future.result()
            status = result.status
        except Exception as exc:
            self._log(f'Sonuc okunamadi: {exc}')
            status = GoalStatus.STATUS_ABORTED

        if status == GoalStatus.STATUS_SUCCEEDED:
            self._log(f'Waypoint {self.idx + 1} OK')
            self._advance(success=True)
        elif status == GoalStatus.STATUS_CANCELED:
            self._log(f'Waypoint {self.idx + 1} iptal edildi')
            if self.state in (self.STATE_CANCELLED, self.STATE_PAUSED):
                return
            # state RUNNING ise (skip vs.) zaten _on_cmd ilerletti
        else:
            self._log(f'Waypoint {self.idx + 1} BASARISIZ (status={status})')
            self._advance(success=False)

    def _advance(self, *, success: bool):
        if success:
            self.idx += 1
            self.retries_left = self.max_retries
        else:
            if self.retries_left > 0:
                self.retries_left -= 1
                self._log(f'Tekrar deneme {self.max_retries - self.retries_left}/{self.max_retries}')
            else:
                self._log('Tekrar deneme tukendi - sonraki waypoint')
                self.idx += 1
                self.retries_left = self.max_retries
        if self.state == self.STATE_RUNNING:
            self._send_current()


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--waypoints', '-w', default=None,
                        help='Waypoint YAML dosyasi')
    parser.add_argument('--frame', '-f', default='map')
    parser.add_argument('--max-retries', type=int, default=1)
    parsed, ros_args = parser.parse_known_args(args=args if args is not None else sys.argv[1:])

    rclpy.init(args=ros_args)

    if parsed.waypoints is None:
        from ament_index_python.packages import get_package_share_directory
        share = get_package_share_directory('ika_mission')
        parsed.waypoints = os.path.join(share, 'missions', 'test_mission.yaml')

    node = GPSWaypointMission(parsed.waypoints, parsed.frame, parsed.max_retries)
    node.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
