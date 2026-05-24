"""IKA - Terrain Perception Node.

Lifecycle node. Depth kamera + IMU + camera_info verilerini fuze ederek:
 - DROPOFF_DANGER : Cukur / dusme kenari
 - IMPASSABLE     : Egim cok dik
 - CAUTION        : Yavaslanmali rampa
 - SAFE           : Sorun yok
 - UNKNOWN        : Veri yetersiz

durumlarindan birini yayimlar.

  /terrain_obstacles  (nav_msgs/OccupancyGrid)  -> costmap layer
  /terrain_state      (std_msgs/String, JSON)   -> Safety Supervisor

Algoritma:
  depth_image + intrinsics -> 3D nokta bulutu (optical frame)
  -> base_link'e tasi (URDF kamera pozisyonu)
  -> RANSAC ile zemin duzlemi
  -> Egim aci + cukur riski + max engel yuksekligi
  -> Siniflandirma

IMU pitch'i siniflandirmada ek bir kontrol olarak kullanilabilir
(rampa uzerindeyken kamera optigi yaniltici olabilir).
"""
import json
import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from sensor_msgs.msg import Image, Imu, CameraInfo
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String

from ika_terrain.ground_plane import (
    depth_to_points, optical_to_base, analyze_ground, TerrainReport,
)

try:
    from diagnostic_updater import Updater
    from diagnostic_msgs.msg import DiagnosticStatus
    _DIAG_OK = True
except ImportError:
    _DIAG_OK = False
    DiagnosticStatus = None


class TerrainPerceptionNode(LifecycleNode):

    def __init__(self):
        super().__init__('terrain_perception')
        self._declare_params()

        self.latest_imu: Optional[Imu] = None
        self.camera_info: Optional[CameraInfo] = None
        self._last_depth_time = None
        self._last_report: Optional[TerrainReport] = None

        self.terrain_pub = None
        self.state_pub = None
        self.depth_sub = None
        self.imu_sub = None
        self.cam_info_sub = None
        self._timer = None

    def _declare_params(self):
        self.declare_parameter('dropoff_depth_threshold_m', 0.15)
        self.declare_parameter('dropoff_lookout_distance_m', 0.60)
        self.declare_parameter('ground_plane_fit_tolerance_m', 0.04)
        self.declare_parameter('max_safe_slope_deg', 15.0)
        self.declare_parameter('max_caution_slope_deg', 25.0)
        self.declare_parameter('max_step_height_m', 0.04)
        self.declare_parameter('terrain_confidence_threshold', 0.6)
        self.declare_parameter('terrain_slowdown_speed_mps', 0.10)
        self.declare_parameter('costmap_resolution', 0.05)
        self.declare_parameter('obstacle_decay_time_s', 2.0)
        self.declare_parameter('publish_rate_hz', 5.0)

        # Kamera montaji (URDF ile uyumlu)
        self.declare_parameter('camera_pitch', 0.15)
        self.declare_parameter('camera_x', 0.10)
        self.declare_parameter('camera_z', 0.15)

        # Hizli isleme icin alt-orneklem
        self.declare_parameter('depth_stride', 4)
        self.declare_parameter('ransac_iterations', 60)

        self.declare_parameter('depth_topic', '/oak/depth/image_raw')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('camera_info_topic', '/oak/stereo/camera_info')

    # ---- lifecycle -----------------------------------------------------
    def on_configure(self, state):
        self.get_logger().info('TerrainNode: configure')
        depth_topic = self.get_parameter('depth_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        cam_info_topic = self.get_parameter('camera_info_topic').value

        self.depth_sub = self.create_subscription(
            Image, depth_topic, self._on_depth, 5)
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self._on_imu, 20)
        self.cam_info_sub = self.create_subscription(
            CameraInfo, cam_info_topic, self._on_cam_info, 1)

        self.terrain_pub = self.create_publisher(
            OccupancyGrid, '/terrain_obstacles', 10)
        self.state_pub = self.create_publisher(String, '/terrain_state', 10)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self._timer = self.create_timer(1.0 / rate, self._tick)

        # Diagnostics
        self._updater = None
        if _DIAG_OK:
            self._updater = Updater(self)
            self._updater.setHardwareID('ika_terrain')
            self._updater.add('Terrain durumu', self._diag_terrain)
            self.create_timer(1.0, lambda: self._updater.update())

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('TerrainNode: activate')
        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info('TerrainNode: deactivate')
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self.get_logger().info('TerrainNode: cleanup')
        for sub in (self.depth_sub, self.imu_sub, self.cam_info_sub):
            if sub is not None:
                self.destroy_subscription(sub)
        for pub in (self.terrain_pub, self.state_pub):
            if pub is not None:
                self.destroy_publisher(pub)
        if self._timer is not None:
            self.destroy_timer(self._timer)
        self.depth_sub = self.imu_sub = self.cam_info_sub = None
        self.terrain_pub = self.state_pub = None
        self._timer = None
        return TransitionCallbackReturn.SUCCESS

    # ---- callback'ler --------------------------------------------------
    def _on_imu(self, msg: Imu):
        self.latest_imu = msg

    def _on_cam_info(self, msg: CameraInfo):
        self.camera_info = msg

    def _on_depth(self, msg: Image):
        depth = self._decode_depth(msg)
        if depth is None or self.camera_info is None:
            self._publish_unknown(reason='no_data')
            return

        fx = float(self.camera_info.k[0])
        fy = float(self.camera_info.k[4])
        cx = float(self.camera_info.k[2])
        cy = float(self.camera_info.k[5])
        if fx <= 0 or fy <= 0:
            self._publish_unknown(reason='bad_intrinsics')
            return

        pts_opt = depth_to_points(
            depth, fx, fy, cx, cy,
            stride=int(self.get_parameter('depth_stride').value),
        )
        if pts_opt.shape[0] < 100:
            self._publish_unknown(reason='too_few_points')
            return

        pts_base = optical_to_base(
            pts_opt,
            camera_pitch=float(self.get_parameter('camera_pitch').value),
            camera_x=float(self.get_parameter('camera_x').value),
            camera_z=float(self.get_parameter('camera_z').value),
        )

        report = analyze_ground(
            pts_base,
            tolerance=float(self.get_parameter('ground_plane_fit_tolerance_m').value),
            safe_slope_deg=float(self.get_parameter('max_safe_slope_deg').value),
            caution_slope_deg=float(self.get_parameter('max_caution_slope_deg').value),
            dropoff_depth_threshold_m=float(self.get_parameter('dropoff_depth_threshold_m').value),
            lookout_distance_m=float(self.get_parameter('dropoff_lookout_distance_m').value),
            max_step_height_m=float(self.get_parameter('max_step_height_m').value),
            confidence_threshold=float(self.get_parameter('terrain_confidence_threshold').value),
        )

        # IMU pitch ile basit tutarlilik kontrolu - aracin gercekten egimde
        # oldugunu kamera goruyorsa, IMU da onaylamali. Aksi durumda CAUTION'a dus.
        if report.classification in ('SAFE', 'CAUTION') and self.latest_imu is not None:
            imu_pitch_deg = self._imu_pitch_deg()
            cam_slope = report.slope_deg
            mismatch = abs(imu_pitch_deg - cam_slope)
            if mismatch > 15.0 and report.classification == 'SAFE':
                report = TerrainReport(
                    classification='CAUTION', slope_deg=report.slope_deg,
                    dropoff_risk=report.dropoff_risk,
                    max_step_height_m=report.max_step_height_m,
                    confidence=report.confidence * 0.7,
                )

        self._last_depth_time = self.get_clock().now()
        self._last_report = report
        self.terrain_pub.publish(self._build_grid(report.classification))
        self._publish_state(report)

    def _tick(self):
        decay = float(self.get_parameter('obstacle_decay_time_s').value)
        if self._last_depth_time is None:
            self._publish_unknown(reason='no_depth_yet')
            return
        elapsed = (self.get_clock().now() - self._last_depth_time).nanoseconds / 1e9
        if elapsed > decay:
            self._publish_unknown(reason='depth_stale')

    # ---- helper'lar ----------------------------------------------------
    def _decode_depth(self, msg: Image) -> Optional[np.ndarray]:
        if msg.encoding not in ('16UC1', 'mono16'):
            self.get_logger().warn(
                f'Beklenmedik depth encoding: {msg.encoding} (16UC1 bekleniyor)',
                throttle_duration_sec=5.0,
            )
            return None
        try:
            raw = np.frombuffer(msg.data, dtype=np.uint16)
            return raw.reshape(msg.height, msg.width).astype(np.float32) / 1000.0
        except Exception as exc:
            self.get_logger().error(f'Depth decode hatasi: {exc}')
            return None

    def _imu_pitch_deg(self) -> float:
        q = self.latest_imu.orientation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = max(-1.0, min(1.0, sinp))
        return math.degrees(math.asin(sinp))

    def _build_grid(self, terrain_class: str) -> OccupancyGrid:
        grid = OccupancyGrid()
        grid.header.frame_id = 'base_link'
        grid.header.stamp = self.get_clock().now().to_msg()
        res = float(self.get_parameter('costmap_resolution').value)
        lookout = float(self.get_parameter('dropoff_lookout_distance_m').value)
        size = max(4, int(lookout / res))
        grid.info.resolution = res
        grid.info.width = size
        grid.info.height = size
        grid.info.origin.position.x = 0.0
        grid.info.origin.position.y = -(size * res) / 2.0
        cost = {
            'DROPOFF_DANGER': 100,
            'IMPASSABLE': 100,
            'CAUTION': 60,
            'UNKNOWN': 30,
            'SAFE': 0,
        }.get(terrain_class, 0)
        grid.data = [cost] * (size * size)
        return grid

    def _publish_unknown(self, *, reason: str):
        rep = TerrainReport('UNKNOWN', 0.0, False, 0.0, 0.0)
        if self.state_pub is None:
            return
        msg = String()
        payload = self._report_payload(rep)
        payload['reason'] = reason
        msg.data = json.dumps(payload)
        self.state_pub.publish(msg)
        if self.terrain_pub is not None:
            self.terrain_pub.publish(self._build_grid('UNKNOWN'))

    def _publish_state(self, rep: TerrainReport):
        if self.state_pub is None:
            return
        msg = String()
        msg.data = json.dumps(self._report_payload(rep))
        self.state_pub.publish(msg)

    def _report_payload(self, rep: TerrainReport) -> dict:
        return {
            'class': rep.classification,
            'slope_deg': rep.slope_deg,
            'dropoff_risk': bool(rep.dropoff_risk),
            'max_step_m': rep.max_step_height_m,
            'confidence': rep.confidence,
        }

    # ---- diagnostics ---------------------------------------------------
    def _diag_terrain(self, stat):
        if self._last_report is None:
            stat.summary(DiagnosticStatus.WARN, 'Henuz veri yok')
            return stat
        rep = self._last_report
        if rep.classification in ('DROPOFF_DANGER', 'IMPASSABLE'):
            stat.summary(DiagnosticStatus.ERROR, f'{rep.classification}')
        elif rep.classification in ('CAUTION', 'UNKNOWN'):
            stat.summary(DiagnosticStatus.WARN, f'{rep.classification}')
        else:
            stat.summary(DiagnosticStatus.OK, 'SAFE')
        stat.add('class', rep.classification)
        stat.add('slope_deg', f'{rep.slope_deg:.2f}')
        stat.add('dropoff_risk', str(rep.dropoff_risk))
        stat.add('max_step_m', f'{rep.max_step_height_m:.3f}')
        stat.add('confidence', f'{rep.confidence:.3f}')
        return stat


def main(args=None):
    rclpy.init(args=args)
    node = TerrainPerceptionNode()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
