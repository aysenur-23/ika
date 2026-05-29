"""IKA - Sim sentetik nesne tespiti node'u.

Sim'de DL yolunu test etmek icin: /odom_truth (Gazebo yer-gercegi) + parametre
ile tanimli scripted engellerden, kameranin FOV+menziline gore Detection3DArray
uretir. Cikti kontrati gercek dl_perception_node ile AYNI (/detected_objects,
class_id="label:hazard") - yani fusion/costmap/safety hicbir farki gormez.

Gercek dl_perception_node sim'de (depthai/kamera yok) hicbir sey yayinlamaz;
bu node /detected_objects'in tek yayincisi olur. sim_detection.launch.py ile
ana sim stack'inin yaninda calistirilir.
"""
import json
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

from ika_perception_dl.sim_detection import SimObstacle, simulate_detections

try:
    from vision_msgs.msg import (
        Detection3D, Detection3DArray, ObjectHypothesisWithPose,
    )
    _VISION_OK = True
except ImportError:
    _VISION_OK = False
    Detection3DArray = None


class SimDetectionNode(Node):

    def __init__(self):
        super().__init__('sim_detection')
        self.declare_parameter('odom_truth_topic', '/odom_truth')
        self.declare_parameter('detections_topic', '/detected_objects')
        self.declare_parameter('state_topic', '/detection_state')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('hfov_deg', 69.0)
        self.declare_parameter('min_range_m', 0.2)
        self.declare_parameter('max_range_m', 6.0)
        self.declare_parameter('nominal_z', 0.3)
        self.declare_parameter('confidence', 0.95)
        self.declare_parameter('publish_rate_hz', 15.0)
        # Scripted engeller - paralel listeler (ayni uzunlukta olmali)
        self.declare_parameter('obstacle_labels', ['person'])
        self.declare_parameter('obstacle_hazards', ['DYNAMIC'])
        self.declare_parameter('obstacle_x0', [2.0])
        self.declare_parameter('obstacle_y0', [-1.5])
        self.declare_parameter('obstacle_vx', [0.0])
        self.declare_parameter('obstacle_vy', [0.4])

        if not _VISION_OK:
            self.get_logger().error(
                'vision_msgs yok - paketi kur (ros-jazzy-vision-msgs)')

        self._rx = 0.0
        self._ry = 0.0
        self._ryaw = 0.0
        self._have_pose = False
        self._t0 = self.get_clock().now()

        self._det_pub = self.create_publisher(
            Detection3DArray, self.get_parameter('detections_topic').value, 10)
        self._state_pub = self.create_publisher(
            String, self.get_parameter('state_topic').value, 10)
        self.create_subscription(
            Odometry, self.get_parameter('odom_truth_topic').value,
            self._on_odom, 20)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info('Sim detection node hazir.')

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._rx = p.x
        self._ry = p.y
        self._ryaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._have_pose = True

    def _obstacles(self):
        g = self.get_parameter
        labels = list(g('obstacle_labels').value)
        hazards = list(g('obstacle_hazards').value)
        x0 = list(g('obstacle_x0').value)
        y0 = list(g('obstacle_y0').value)
        vx = list(g('obstacle_vx').value)
        vy = list(g('obstacle_vy').value)
        n = min(len(labels), len(hazards), len(x0), len(y0), len(vx), len(vy))
        return [SimObstacle(labels[i], hazards[i], x0[i], y0[i], vx[i], vy[i])
                for i in range(n)]

    def _tick(self):
        if not _VISION_OK or not self._have_pose:
            return
        t = (self.get_clock().now() - self._t0).nanoseconds / 1e9
        dets = simulate_detections(
            self._obstacles(), t, self._rx, self._ry, self._ryaw,
            hfov_rad=math.radians(float(self.get_parameter('hfov_deg').value)),
            min_range_m=float(self.get_parameter('min_range_m').value),
            max_range_m=float(self.get_parameter('max_range_m').value),
            nominal_z=float(self.get_parameter('nominal_z').value),
            confidence=float(self.get_parameter('confidence').value),
        )
        self._publish(dets)

    def _publish(self, dets):
        frame = self.get_parameter('frame_id').value
        stamp = self.get_clock().now().to_msg()
        arr = Detection3DArray()
        arr.header.frame_id = frame
        arr.header.stamp = stamp
        dynamic = 0
        for d in dets:
            det = Detection3D()
            det.header.frame_id = frame
            det.header.stamp = stamp
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = f'{d.label}:{d.hazard}'
            hyp.hypothesis.score = float(d.confidence)
            hyp.pose.pose.position.x = float(d.x)
            hyp.pose.pose.position.y = float(d.y)
            hyp.pose.pose.position.z = float(d.z)
            hyp.pose.pose.orientation.w = 1.0
            det.results.append(hyp)
            det.bbox.center.position.x = float(d.x)
            det.bbox.center.position.y = float(d.y)
            det.bbox.center.position.z = float(d.z)
            arr.detections.append(det)
            if d.hazard == 'DYNAMIC':
                dynamic += 1
        self._det_pub.publish(arr)

        state = String()
        state.data = json.dumps({'count': len(dets), 'dynamic_count': dynamic,
                                 'source': 'sim'})
        self._state_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = SimDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
