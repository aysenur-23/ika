"""IKA - Safety Supervisor.

Lifecycle node. Tum guvenlik kararlarini merkezi olarak verir:

  /cmd_vel_collision  ----+
                          |  filtre  ->  /cmd_vel_safe
  /terrain_state      ----+
  /scan, /depth, /imu --> sensor watchdog -> /e_stop

  /safety_status       JSON statu yayini (diagnostics + RViz icin)
"""
import json

import rclpy
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan, Image
from std_msgs.msg import Bool, String

try:
    from diagnostic_updater import Updater
    from diagnostic_msgs.msg import DiagnosticStatus
    _DIAG_OK = True
except ImportError:
    _DIAG_OK = False
    DiagnosticStatus = None


class SafetySupervisorNode(LifecycleNode):

    def __init__(self):
        super().__init__('safety_supervisor')
        self._declare_params()

        self._cmd_vel_sub = None
        self._terrain_sub = None
        self._scan_sub = None
        self._depth_sub = None
        self._imu_sub = None
        self._cmd_vel_pub = None
        self._e_stop_pub = None
        self._status_pub = None
        self._watchdog_timer = None

    def _declare_params(self):
        self.declare_parameter('lidar_timeout_s', 1.0)
        self.declare_parameter('depth_timeout_s', 1.5)
        self.declare_parameter('imu_timeout_s', 0.5)
        self.declare_parameter('stop_zone_distance_m', 0.25)
        self.declare_parameter('slowdown_zone_distance_m', 0.55)
        self.declare_parameter('slowdown_speed_factor', 0.3)
        self.declare_parameter('terrain_stop_classes',
                               ['DROPOFF_DANGER', 'IMPASSABLE'])
        self.declare_parameter('terrain_slow_classes',
                               ['CAUTION', 'UNKNOWN'])
        self.declare_parameter('cmd_vel_in_topic', '/cmd_vel_collision')
        self.declare_parameter('cmd_vel_out_topic', '/cmd_vel_safe')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('depth_topic', '/oak/depth/image_raw')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('watchdog_rate_hz', 20.0)
        self.declare_parameter('recovery_wait_s', 3.0)

    # ---- lifecycle -----------------------------------------------------
    def on_configure(self, state):
        self.get_logger().info('SafetySupervisor: configure')

        in_topic = self.get_parameter('cmd_vel_in_topic').value
        out_topic = self.get_parameter('cmd_vel_out_topic').value
        scan_topic = self.get_parameter('scan_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        imu_topic = self.get_parameter('imu_topic').value

        self._cmd_vel_sub = self.create_subscription(
            Twist, in_topic, self._on_cmd_vel, 10)
        self._terrain_sub = self.create_subscription(
            String, '/terrain_state', self._on_terrain, 10)
        self._scan_sub = self.create_subscription(
            LaserScan, scan_topic, self._on_scan, 10)
        self._depth_sub = self.create_subscription(
            Image, depth_topic, self._on_depth, 10)
        self._imu_sub = self.create_subscription(
            Imu, imu_topic, self._on_imu, 20)

        self._cmd_vel_pub = self.create_publisher(Twist, out_topic, 10)
        self._e_stop_pub = self.create_publisher(Bool, '/e_stop', 10)
        self._status_pub = self.create_publisher(String, '/safety_status', 10)

        now = self.get_clock().now()
        self.terrain_class = 'UNKNOWN'
        self.last_scan_time = now
        self.last_depth_time = now
        self.last_imu_time = now
        self.e_stop_active = False

        rate = float(self.get_parameter('watchdog_rate_hz').value)
        self._watchdog_timer = self.create_timer(1.0 / rate, self._watchdog)

        # Diagnostics
        self._updater = None
        if _DIAG_OK:
            self._updater = Updater(self)
            self._updater.setHardwareID('ika_safety_supervisor')
            self._updater.add('Sensor watchdog', self._diag_sensors)
            self._updater.add('Terrain durumu', self._diag_terrain)
            self.create_timer(1.0, lambda: self._updater.update())

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state):
        self.get_logger().info('SafetySupervisor: activate')
        return super().on_activate(state)

    def on_deactivate(self, state):
        return super().on_deactivate(state)

    # ---- veri callback'leri -------------------------------------------
    def _on_terrain(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.terrain_class = data.get('class', 'UNKNOWN')
        except json.JSONDecodeError:
            self.terrain_class = 'UNKNOWN'

    def _on_scan(self, _msg: LaserScan):
        self.last_scan_time = self.get_clock().now()

    def _on_depth(self, _msg: Image):
        self.last_depth_time = self.get_clock().now()

    def _on_imu(self, _msg: Imu):
        self.last_imu_time = self.get_clock().now()

    # ---- filtre + watchdog --------------------------------------------
    def _on_cmd_vel(self, msg: Twist):
        if self._cmd_vel_pub is None:
            return
        if self.e_stop_active:
            self._publish_stop()
            return

        stop_classes = list(self.get_parameter('terrain_stop_classes').value)
        slow_classes = list(self.get_parameter('terrain_slow_classes').value)

        if self.terrain_class in stop_classes:
            self._publish_stop()
            self.get_logger().warn(
                f'Terrain DUR: {self.terrain_class}', throttle_duration_sec=1.0)
            return

        if self.terrain_class in slow_classes:
            factor = float(self.get_parameter('slowdown_speed_factor').value)
            filtered = Twist()
            filtered.linear.x = msg.linear.x * factor
            filtered.linear.y = msg.linear.y * factor
            filtered.angular.z = msg.angular.z * factor
            self._cmd_vel_pub.publish(filtered)
            return

        self._cmd_vel_pub.publish(msg)

    def _watchdog(self):
        now = self.get_clock().now()

        def age_s(t):
            return (now - t).nanoseconds / 1e9

        scan_age = age_s(self.last_scan_time)
        depth_age = age_s(self.last_depth_time)
        imu_age = age_s(self.last_imu_time)

        lidar_ok = scan_age < self.get_parameter('lidar_timeout_s').value
        depth_ok = depth_age < self.get_parameter('depth_timeout_s').value
        imu_ok = imu_age < self.get_parameter('imu_timeout_s').value

        prev = self.e_stop_active
        self.e_stop_active = not (lidar_ok and depth_ok and imu_ok)

        e_stop_msg = Bool()
        e_stop_msg.data = self.e_stop_active
        if self._e_stop_pub is not None:
            self._e_stop_pub.publish(e_stop_msg)

        if self.e_stop_active:
            if not prev:
                self.get_logger().error(
                    f'SENSOR TIMEOUT - lidar:{lidar_ok} depth:{depth_ok} imu:{imu_ok}')
            self._publish_stop()

        status = String()
        status.data = json.dumps({
            'e_stop': self.e_stop_active,
            'terrain_class': self.terrain_class,
            'lidar_ok': lidar_ok,
            'depth_ok': depth_ok,
            'imu_ok': imu_ok,
            'scan_age_s': round(scan_age, 3),
            'depth_age_s': round(depth_age, 3),
            'imu_age_s': round(imu_age, 3),
        })
        if self._status_pub is not None:
            self._status_pub.publish(status)

    def _publish_stop(self):
        if self._cmd_vel_pub is not None:
            self._cmd_vel_pub.publish(Twist())

    # ---- diagnostics ---------------------------------------------------
    def _diag_sensors(self, stat):
        now = self.get_clock().now()
        scan_age = (now - self.last_scan_time).nanoseconds / 1e9
        depth_age = (now - self.last_depth_time).nanoseconds / 1e9
        imu_age = (now - self.last_imu_time).nanoseconds / 1e9

        lidar_to = self.get_parameter('lidar_timeout_s').value
        depth_to = self.get_parameter('depth_timeout_s').value
        imu_to = self.get_parameter('imu_timeout_s').value

        lidar_ok = scan_age < lidar_to
        depth_ok = depth_age < depth_to
        imu_ok = imu_age < imu_to

        if lidar_ok and depth_ok and imu_ok:
            stat.summary(DiagnosticStatus.OK, 'Tum sensorler taze')
        else:
            missing = []
            if not lidar_ok: missing.append('lidar')
            if not depth_ok: missing.append('depth')
            if not imu_ok:   missing.append('imu')
            stat.summary(DiagnosticStatus.ERROR, f'Eksik/eski: {",".join(missing)}')

        stat.add('scan_age_s', f'{scan_age:.3f}')
        stat.add('depth_age_s', f'{depth_age:.3f}')
        stat.add('imu_age_s', f'{imu_age:.3f}')
        stat.add('e_stop', str(self.e_stop_active))
        return stat

    def _diag_terrain(self, stat):
        cls = self.terrain_class
        stop_classes = list(self.get_parameter('terrain_stop_classes').value)
        slow_classes = list(self.get_parameter('terrain_slow_classes').value)
        if cls in stop_classes:
            stat.summary(DiagnosticStatus.ERROR, f'Terrain DUR: {cls}')
        elif cls in slow_classes:
            stat.summary(DiagnosticStatus.WARN, f'Terrain yavasla: {cls}')
        else:
            stat.summary(DiagnosticStatus.OK, f'Terrain: {cls}')
        stat.add('class', cls)
        return stat


def main(args=None):
    rclpy.init(args=args)
    node = SafetySupervisorNode()
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
