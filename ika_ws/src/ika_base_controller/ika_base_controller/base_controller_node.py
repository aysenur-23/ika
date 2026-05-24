"""IKA - Base Controller Node.

Pi <-> Arduino seri kopru.
/cmd_vel_safe (Twist) -> sol/sag teker hizi -> JSON satir -> Arduino.
/e_stop (Bool=True) -> Motorlari durdur.

Encoder yok: Bu node odometri YAYIMLAMAZ. Encoder eklendiginde
/odom publisher buraya tasinabilir.
"""
import json
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

try:
    import serial
except ImportError:
    serial = None

try:
    from diagnostic_updater import Updater, DiagnosticStatusWrapper
    from diagnostic_msgs.msg import DiagnosticStatus
    _DIAG_OK = True
except ImportError:
    _DIAG_OK = False
    DiagnosticStatus = None


class BaseControllerNode(Node):

    def __init__(self):
        super().__init__('ika_base_controller')

        self.declare_parameter('serial_port', '/dev/ika_arduino')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_base', 0.30)
        self.declare_parameter('max_linear_speed', 0.30)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('dry_run', False)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_safe')

        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        self.dry_run = self.get_parameter('dry_run').value

        self._serial = None
        self._lock = threading.Lock()
        self._open_serial()

        cmd_topic = self.get_parameter('cmd_vel_topic').value
        self.cmd_vel_sub = self.create_subscription(
            Twist, cmd_topic, self._on_cmd_vel, 10)
        self.e_stop_sub = self.create_subscription(
            Bool, '/e_stop', self._on_e_stop, 10)

        self.status_pub = self.create_publisher(String, '/base_controller_status', 10)

        self._last_cmd = (0.0, 0.0)
        self._last_cmd_time = self.get_clock().now()
        self._e_stop = False

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._tick)

        # Diagnostics
        self._updater = None
        if _DIAG_OK:
            self._updater = Updater(self)
            self._updater.setHardwareID('ika_base_controller')
            self._updater.add('Arduino seri', self._diag_serial)
            self._updater.add('cmd_vel watchdog', self._diag_watchdog)
            self.create_timer(1.0, lambda: self._updater.update())

        self.get_logger().info(
            f'BaseController hazir | port={self.port} baud={self.baud} '
            f'dry_run={self.dry_run} topic={cmd_topic}')

    # ---- seri port -----------------------------------------------------
    def _open_serial(self):
        if self.dry_run:
            self.get_logger().warn('dry_run=true - seri port acilmayacak')
            return
        if serial is None:
            self.get_logger().error("pyserial bulunamadi - 'pip install pyserial'")
            return
        try:
            self._serial = serial.Serial(self.port, self.baud, timeout=0.05)
            self.get_logger().info(f'Arduino acildi: {self.port}@{self.baud}')
        except Exception as exc:  # SerialException dahil
            self.get_logger().error(f'Arduino acilamadi ({self.port}): {exc}')
            self._serial = None

    def _send(self, v_left: float, v_right: float):
        max_v = self.get_parameter('max_linear_speed').value
        v_left = max(-max_v, min(max_v, v_left))
        v_right = max(-max_v, min(max_v, v_right))
        payload = json.dumps({'l': round(v_left, 3), 'r': round(v_right, 3)}) + '\n'
        if self._serial is None:
            return
        try:
            with self._lock:
                self._serial.write(payload.encode('ascii'))
        except Exception as exc:
            self.get_logger().error(f'Seri yazma hatasi: {exc}')
            self._serial = None

    # ---- callback'ler --------------------------------------------------
    def _on_cmd_vel(self, msg: Twist):
        wheel_base = self.get_parameter('wheel_base').value
        v_left = msg.linear.x - (msg.angular.z * wheel_base / 2.0)
        v_right = msg.linear.x + (msg.angular.z * wheel_base / 2.0)
        self._last_cmd = (v_left, v_right)
        self._last_cmd_time = self.get_clock().now()

    def _on_e_stop(self, msg: Bool):
        self._e_stop = bool(msg.data)
        if self._e_stop:
            self.get_logger().warn('E-STOP aktif')

    # ---- periyodik gonderme -------------------------------------------
    def _tick(self):
        timeout = self.get_parameter('cmd_vel_timeout').value
        now = self.get_clock().now()
        elapsed = (now - self._last_cmd_time).nanoseconds / 1e9

        if self._e_stop or elapsed > timeout:
            v_left, v_right = 0.0, 0.0
        else:
            v_left, v_right = self._last_cmd

        self._send(v_left, v_right)

        status = String()
        status.data = json.dumps({
            'connected': self._serial is not None,
            'e_stop': self._e_stop,
            'cmd_age_s': round(elapsed, 3),
            'v_left': round(v_left, 3),
            'v_right': round(v_right, 3),
        })
        self.status_pub.publish(status)

    # ---- diagnostics ---------------------------------------------------
    def _diag_serial(self, stat):
        if self.dry_run:
            stat.summary(DiagnosticStatus.WARN, 'dry_run aktif - seri kapali')
        elif self._serial is None:
            stat.summary(DiagnosticStatus.ERROR, f'Seri port acik degil ({self.port})')
        else:
            stat.summary(DiagnosticStatus.OK, f'Bagli: {self.port}@{self.baud}')
        stat.add('port', str(self.port))
        stat.add('baud', str(self.baud))
        stat.add('dry_run', str(self.dry_run))
        return stat

    def _diag_watchdog(self, stat):
        now = self.get_clock().now()
        age = (now - self._last_cmd_time).nanoseconds / 1e9
        timeout = float(self.get_parameter('cmd_vel_timeout').value)
        if self._e_stop:
            stat.summary(DiagnosticStatus.WARN, 'E-STOP aktif')
        elif age > timeout:
            stat.summary(DiagnosticStatus.WARN, f'cmd_vel zaman asimi ({age:.2f}s)')
        else:
            stat.summary(DiagnosticStatus.OK, f'cmd_vel taze ({age:.2f}s)')
        stat.add('age_s', f'{age:.3f}')
        stat.add('e_stop', str(self._e_stop))
        stat.add('v_left', f'{self._last_cmd[0]:.3f}')
        stat.add('v_right', f'{self._last_cmd[1]:.3f}')
        return stat

    def destroy_node(self):
        if self._serial is not None:
            try:
                self._send(0.0, 0.0)
                self._serial.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BaseControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
