"""IKA - Hibrit Hazard Fusion Node.

Lifecycle node. DL nesne tespiti ile RANSAC terrain'i birlestirir:

  /terrain_state    (std_msgs/String JSON)         --+
                                                      |-> fuse -> kararlar
  /detected_objects (vision_msgs/Detection3DArray) --+

  Cikti:
    /hazard_state        (std_msgs/String JSON)    -> Safety Supervisor
    /detection_obstacles (nav_msgs/OccupancyGrid)  -> costmap detection_layer

Fuzyon mantigi hazard_fusion cekirdeginde (ROS'suz, test edilebilir).
"""
import json
import math
from typing import Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid

from ika_fusion.hazard_fusion import (
    DetectedObject, fuse_hazard, decision_payload, detections_to_grid,
)

try:
    from vision_msgs.msg import Detection3DArray
    _VISION_OK = True
except ImportError:
    _VISION_OK = False
    Detection3DArray = None

try:
    from diagnostic_updater import Updater
    from diagnostic_msgs.msg import DiagnosticStatus
    _DIAG_OK = True
except ImportError:
    _DIAG_OK = False
    DiagnosticStatus = None


class FusionNode(LifecycleNode):

    def __init__(self):
        super().__init__('hazard_fusion')
        self._declare_params()

        self._terrain_class = 'UNKNOWN'
        self._detections = []
        self._last_det_time = None
        self._last_decision = None

        self._terrain_sub = None
        self._det_sub = None
        self._hazard_pub = None
        self._grid_pub = None
        self._timer = None

    def _declare_params(self):
        self.declare_parameter('terrain_stop_classes',
                               ['DROPOFF_DANGER', 'IMPASSABLE'])
        self.declare_parameter('terrain_slow_classes', ['CAUTION', 'UNKNOWN'])
        self.declare_parameter('dynamic_stop_range_m', 0.8)
        self.declare_parameter('dynamic_slow_range_m', 2.0)

        # Grid
        self.declare_parameter('grid_resolution', 0.05)
        self.declare_parameter('grid_size_cells', 80)
        self.declare_parameter('obstacle_radius_m', 0.15)
        self.declare_parameter('dynamic_cost', 100)
        self.declare_parameter('static_cost', 100)
        self.declare_parameter('detection_decay_time_s', 1.0)

        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('terrain_state_topic', '/terrain_state')
        self.declare_parameter('detections_topic', '/detected_objects')
        self.declare_parameter('hazard_state_topic', '/hazard_state')
        self.declare_parameter('detection_grid_topic', '/detection_obstacles')

    # ---- lifecycle -----------------------------------------------------
    def on_configure(self, state):
        self.get_logger().info('FusionNode: configure')
        if not _VISION_OK:
            self.get_logger().error(
                'vision_msgs yok - paketi kur (ros-jazzy-vision-msgs)')
            return TransitionCallbackReturn.FAILURE

        self._terrain_sub = self.create_subscription(
            String, self.get_parameter('terrain_state_topic').value,
            self._on_terrain, 10)
        self._det_sub = self.create_subscription(
            Detection3DArray, self.get_parameter('detections_topic').value,
            self._on_detections, 10)

        self._hazard_pub = self.create_publisher(
            String, self.get_parameter('hazard_state_topic').value, 10)
        self._grid_pub = self.create_publisher(
            OccupancyGrid, self.get_parameter('detection_grid_topic').value, 10)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self._timer = self.create_timer(1.0 / rate, self._tick)

        self._updater = None
        if _DIAG_OK:
            self._updater = Updater(self)
            self._updater.setHardwareID('ika_fusion')
            self._updater.add('Hazard fuzyon', self._diag)
            self.create_timer(1.0, lambda: self._updater.update())

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('FusionNode: activate')
        return super().on_activate(state)

    def on_deactivate(self, state):
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        for sub in (self._terrain_sub, self._det_sub):
            if sub is not None:
                self.destroy_subscription(sub)
        for pub in (self._hazard_pub, self._grid_pub):
            if pub is not None:
                self.destroy_publisher(pub)
        if self._timer is not None:
            self.destroy_timer(self._timer)
        self._terrain_sub = self._det_sub = None
        self._hazard_pub = self._grid_pub = None
        self._timer = None
        return TransitionCallbackReturn.SUCCESS

    # ---- callback'ler --------------------------------------------------
    def _on_terrain(self, msg: String):
        try:
            data = json.loads(msg.data)
            self._terrain_class = data.get('class', 'UNKNOWN')
        except json.JSONDecodeError:
            self._terrain_class = 'UNKNOWN'

    def _on_detections(self, msg):
        dets = []
        for d in msg.detections:
            if not d.results:
                continue
            hyp = d.results[0]
            class_id = hyp.hypothesis.class_id
            # class_id formati "label:hazard" (dl_perception_node uretir)
            if ':' in class_id:
                label, hazard = class_id.rsplit(':', 1)
            else:
                label, hazard = class_id, 'STATIC'
            pos = hyp.pose.pose.position
            x, y = float(pos.x), float(pos.y)
            dets.append(DetectedObject(
                x=x, y=y, hazard=hazard, label=label,
                confidence=float(hyp.hypothesis.score),
                range_m=math.hypot(x, y),
            ))
        self._detections = dets
        self._last_det_time = self.get_clock().now()

    # ---- fuzyon + yayim ------------------------------------------------
    def _tick(self):
        # Tespit bayatladiysa bos kabul et (dinamik nesneler hizla eskimeli)
        dets = self._detections
        decay = float(self.get_parameter('detection_decay_time_s').value)
        if self._last_det_time is not None:
            age = (self.get_clock().now() - self._last_det_time).nanoseconds / 1e9
            if age > decay:
                dets = []
        else:
            dets = []

        decision = fuse_hazard(
            self._terrain_class, dets,
            terrain_stop_classes=list(
                self.get_parameter('terrain_stop_classes').value),
            terrain_slow_classes=list(
                self.get_parameter('terrain_slow_classes').value),
            dynamic_stop_range_m=float(
                self.get_parameter('dynamic_stop_range_m').value),
            dynamic_slow_range_m=float(
                self.get_parameter('dynamic_slow_range_m').value),
        )
        self._last_decision = decision

        if self._hazard_pub is not None:
            msg = String()
            msg.data = json.dumps(decision_payload(decision))
            self._hazard_pub.publish(msg)

        if self._grid_pub is not None:
            self._grid_pub.publish(self._build_grid(dets))

    def _build_grid(self, dets) -> OccupancyGrid:
        data, meta = detections_to_grid(
            dets,
            resolution=float(self.get_parameter('grid_resolution').value),
            size_cells=int(self.get_parameter('grid_size_cells').value),
            obstacle_radius_m=float(
                self.get_parameter('obstacle_radius_m').value),
            dynamic_cost=int(self.get_parameter('dynamic_cost').value),
            static_cost=int(self.get_parameter('static_cost').value),
        )
        grid = OccupancyGrid()
        grid.header.frame_id = self.get_parameter('frame_id').value
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = meta['resolution']
        grid.info.width = meta['size_cells']
        grid.info.height = meta['size_cells']
        grid.info.origin.position.x = meta['origin_x']
        grid.info.origin.position.y = meta['origin_y']
        grid.info.origin.orientation.w = 1.0
        grid.data = data
        return grid

    # ---- diagnostics ---------------------------------------------------
    def _diag(self, stat):
        d = self._last_decision
        if d is None:
            stat.summary(DiagnosticStatus.WARN, 'Henuz karar yok')
            return stat
        level = {
            'STOP': DiagnosticStatus.ERROR,
            'SLOW': DiagnosticStatus.WARN,
            'CLEAR': DiagnosticStatus.OK,
        }.get(d.action, DiagnosticStatus.OK)
        stat.summary(level, f'{d.action} ({",".join(d.sources) or "temiz"})')
        stat.add('action', d.action)
        stat.add('terrain_class', d.terrain_class)
        stat.add('dynamic_count', str(d.dynamic_count))
        stat.add('reasons', ';'.join(d.reasons))
        return stat


def main(args=None):
    rclpy.init(args=args)
    node = FusionNode()
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
