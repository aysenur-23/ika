"""IKA - DL Perception Node (OAK-D Lite VPU spatial detection).

Lifecycle node. OAK-D Lite uzerinde SpatialDetectionNetwork (MobileNet veya
YOLO) calistirir; her tespit icin metrik 3B konum (kamera optical frame) alir,
base_link'e tasir ve yayar:

  /detected_objects   (vision_msgs/Detection3DArray)  -> fusion node + costmap
  /detection_state    (std_msgs/String, JSON ozet)    -> fusion / diagnostics

Agir NN cikarimi kameranin VPU'sunda kosar; Pi yalniz hafif son-isleme yapar
(detector_postprocess). depthai/vision_msgs yoksa node yine import edilebilir
ama aktive olunca uyari verir (sim'de veya kamerasiz Pi'de fail-safe).
"""
import json
from typing import Optional

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from std_msgs.msg import String

from ika_perception_dl.detector_postprocess import (
    RawSpatialDetection, process_detections, summarize,
)

try:
    import depthai as dai
    _DEPTHAI_OK = True
except ImportError:
    _DEPTHAI_OK = False
    dai = None

try:
    from vision_msgs.msg import (
        Detection3D, Detection3DArray, ObjectHypothesisWithPose,
    )
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


class DLPerceptionNode(LifecycleNode):

    def __init__(self):
        super().__init__('dl_perception')
        self._declare_params()

        self._device = None
        self._det_queue = None
        self._det_pub = None
        self._state_pub = None
        self._timer = None
        self._last_detections = []
        self._last_summary = {'count': 0, 'dynamic_count': 0}

    def _declare_params(self):
        # Model
        self.declare_parameter('model_type', 'mobilenet')       # mobilenet | yolo
        self.declare_parameter('model_blob_path', '')
        self.declare_parameter('nn_input_width', 300)
        self.declare_parameter('nn_input_height', 300)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('bounding_box_scale_factor', 0.5)
        self.declare_parameter('depth_lower_threshold_mm', 200)
        self.declare_parameter('depth_upper_threshold_mm', 6000)

        # YOLO'ya ozel (model_type=yolo iken)
        self.declare_parameter('yolo_num_classes', 80)
        self.declare_parameter('yolo_coordinate_size', 4)
        self.declare_parameter('yolo_iou_threshold', 0.5)
        self.declare_parameter('yolo_anchors', [0.0])
        self.declare_parameter('yolo_anchor_mask_keys', [''])
        self.declare_parameter('yolo_anchor_mask_vals', [0])

        # Label -> hazard
        self.declare_parameter('label_names', ['background'])
        self.declare_parameter('dynamic_labels', ['person'])
        self.declare_parameter('static_labels', [''])
        self.declare_parameter('ignore_labels', ['background'])
        self.declare_parameter('default_hazard', 'STATIC')

        # Filtre / menzil
        self.declare_parameter('min_range_m', 0.2)
        self.declare_parameter('max_range_m', 6.0)

        # Kamera montaji (URDF + terrain ile uyumlu)
        self.declare_parameter('camera_pitch', 0.15)
        self.declare_parameter('camera_x', 0.10)
        self.declare_parameter('camera_z', 0.15)

        # Yayim
        self.declare_parameter('poll_rate_hz', 15.0)
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('detections_topic', '/detected_objects')
        self.declare_parameter('state_topic', '/detection_state')

    # ---- lifecycle -----------------------------------------------------
    def on_configure(self, state):
        self.get_logger().info('DLPerception: configure')

        if not _VISION_OK:
            self.get_logger().error(
                'vision_msgs yok - paketi kur (ros-jazzy-vision-msgs)')
            return TransitionCallbackReturn.FAILURE

        det_topic = self.get_parameter('detections_topic').value
        state_topic = self.get_parameter('state_topic').value
        self._det_pub = self.create_publisher(Detection3DArray, det_topic, 10)
        self._state_pub = self.create_publisher(String, state_topic, 10)

        rate = float(self.get_parameter('poll_rate_hz').value)
        self._timer = self.create_timer(1.0 / rate, self._poll)

        self._updater = None
        if _DIAG_OK:
            self._updater = Updater(self)
            self._updater.setHardwareID('ika_perception_dl')
            self._updater.add('DL tespit', self._diag_detections)
            self.create_timer(1.0, lambda: self._updater.update())

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('DLPerception: activate')
        if not _DEPTHAI_OK:
            self.get_logger().warn(
                'depthai yok - OAK-D baglanamaz, tespit yayini bos kalacak '
                '(sim/kamerasiz mod)')
        else:
            try:
                pipeline = self._build_pipeline()
                self._device = dai.Device(pipeline)
                self._det_queue = self._device.getOutputQueue(
                    'detections', maxSize=4, blocking=False)
                self.get_logger().info('OAK-D baglandi, spatial NN aktif')
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'OAK-D baslatma hatasi: {exc}')
                self._device = None
                self._det_queue = None
        return super().on_activate(state)

    def on_deactivate(self, state):
        self.get_logger().info('DLPerception: deactivate')
        self._close_device()
        return super().on_deactivate(state)

    def on_cleanup(self, state):
        self._close_device()
        if self._timer is not None:
            self.destroy_timer(self._timer)
            self._timer = None
        if self._det_pub is not None:
            self.destroy_publisher(self._det_pub)
            self._det_pub = None
        if self._state_pub is not None:
            self.destroy_publisher(self._state_pub)
            self._state_pub = None
        return TransitionCallbackReturn.SUCCESS

    def _close_device(self):
        if self._device is not None:
            try:
                self._device.close()
            except Exception:  # noqa: BLE001
                pass
        self._device = None
        self._det_queue = None

    # ---- depthai pipeline ---------------------------------------------
    def _build_pipeline(self):
        w = int(self.get_parameter('nn_input_width').value)
        h = int(self.get_parameter('nn_input_height').value)
        blob = self.get_parameter('model_blob_path').value
        conf = float(self.get_parameter('confidence_threshold').value)
        model_type = self.get_parameter('model_type').value

        pipeline = dai.Pipeline()

        cam_rgb = pipeline.createColorCamera()
        cam_rgb.setPreviewSize(w, h)
        cam_rgb.setResolution(
            dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam_rgb.setInterleaved(False)
        cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)

        mono_left = pipeline.createMonoCamera()
        mono_left.setResolution(
            dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        mono_right = pipeline.createMonoCamera()
        mono_right.setResolution(
            dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        stereo = pipeline.createStereoDepth()
        stereo.setDefaultProfilePreset(
            dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        mono_left.out.link(stereo.left)
        mono_right.out.link(stereo.right)

        if model_type == 'yolo':
            nn = pipeline.createYoloSpatialDetectionNetwork()
            nn.setNumClasses(int(self.get_parameter('yolo_num_classes').value))
            nn.setCoordinateSize(
                int(self.get_parameter('yolo_coordinate_size').value))
            nn.setIouThreshold(
                float(self.get_parameter('yolo_iou_threshold').value))
            anchors = [float(a) for a in
                       self.get_parameter('yolo_anchors').value]
            if anchors and anchors != [0.0]:
                nn.setAnchors(anchors)
            mask = self._yolo_anchor_masks()
            if mask:
                nn.setAnchorMasks(mask)
        else:
            nn = pipeline.createMobileNetSpatialDetectionNetwork()

        nn.setBlobPath(blob)
        nn.setConfidenceThreshold(conf)
        nn.input.setBlocking(False)
        nn.setBoundingBoxScaleFactor(
            float(self.get_parameter('bounding_box_scale_factor').value))
        nn.setDepthLowerThreshold(
            int(self.get_parameter('depth_lower_threshold_mm').value))
        nn.setDepthUpperThreshold(
            int(self.get_parameter('depth_upper_threshold_mm').value))

        cam_rgb.preview.link(nn.input)
        stereo.depth.link(nn.inputDepth)

        xout = pipeline.createXLinkOut()
        xout.setStreamName('detections')
        nn.out.link(xout.input)

        return pipeline

    def _yolo_anchor_masks(self) -> dict:
        keys = list(self.get_parameter('yolo_anchor_mask_keys').value)
        vals = list(self.get_parameter('yolo_anchor_mask_vals').value)
        if not keys or keys == [''] or len(keys) != len(vals):
            return {}
        # vals duz liste: her key icin tek deger (basit modeller). Karmasik
        # mask'ler model JSON'undan dogrudan param olarak girilmeli.
        return {k: [int(v)] for k, v in zip(keys, vals)}

    # ---- poll + publish ------------------------------------------------
    def _poll(self):
        if self._det_queue is None:
            return
        in_det = self._det_queue.tryGet()
        if in_det is None:
            return

        raw = []
        for d in in_det.detections:
            sc = d.spatialCoordinates
            raw.append(RawSpatialDetection(
                label_id=int(d.label),
                confidence=float(d.confidence),
                x=float(sc.x) / 1000.0,   # mm -> m
                y=float(sc.y) / 1000.0,
                z=float(sc.z) / 1000.0,
                bbox=(float(d.xmin), float(d.ymin),
                      float(d.xmax), float(d.ymax)),
            ))

        dets = process_detections(
            raw,
            list(self.get_parameter('label_names').value),
            dynamic_labels=list(self.get_parameter('dynamic_labels').value),
            static_labels=list(self.get_parameter('static_labels').value),
            ignore_labels=list(self.get_parameter('ignore_labels').value),
            confidence_threshold=float(
                self.get_parameter('confidence_threshold').value),
            min_range_m=float(self.get_parameter('min_range_m').value),
            max_range_m=float(self.get_parameter('max_range_m').value),
            camera_pitch=float(self.get_parameter('camera_pitch').value),
            camera_x=float(self.get_parameter('camera_x').value),
            camera_z=float(self.get_parameter('camera_z').value),
            default_hazard=self.get_parameter('default_hazard').value,
        )
        self._last_detections = dets
        self._last_summary = summarize(dets)
        self._publish(dets)

    def _publish(self, dets):
        frame = self.get_parameter('frame_id').value
        stamp = self.get_clock().now().to_msg()

        arr = Detection3DArray()
        arr.header.frame_id = frame
        arr.header.stamp = stamp
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
        if self._det_pub is not None:
            self._det_pub.publish(arr)

        if self._state_pub is not None:
            msg = String()
            msg.data = json.dumps(self._last_summary)
            self._state_pub.publish(msg)

    # ---- diagnostics ---------------------------------------------------
    def _diag_detections(self, stat):
        s = self._last_summary
        if not _DEPTHAI_OK or self._device is None:
            stat.summary(DiagnosticStatus.WARN, 'Kamera bagli degil')
        elif s.get('dynamic_count', 0) > 0:
            stat.summary(
                DiagnosticStatus.OK,
                f"{s['dynamic_count']} dinamik nesne")
        else:
            stat.summary(DiagnosticStatus.OK, 'Dinamik nesne yok')
        stat.add('count', str(s.get('count', 0)))
        stat.add('dynamic_count', str(s.get('dynamic_count', 0)))
        stat.add('nearest_dynamic_range_m',
                 str(s.get('nearest_dynamic_range_m')))
        return stat


def main(args=None):
    rclpy.init(args=args)
    node = DLPerceptionNode()
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
