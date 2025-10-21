import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from collections import deque
from typing import Optional
from interfaces.msg import RobotStatus


class HealthAnalyzerNode(Node):
    def __init__(self):
        super().__init__('health_analyzer_node')

        # Declare configurable parameters
        self.declare_parameter('rpm_limit', 2000.0)
        self.declare_parameter('battery_warning_threshold', 25.0)
        self.declare_parameter('battery_critical_threshold', 15.0)
        self.declare_parameter('zero_rpm_timeout', 5.0)
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('history_length', 10)

        # Load parameter values
        self.rpm_limit = float(self.get_parameter('rpm_limit').value)
        self.battery_warning_threshold = float(self.get_parameter('battery_warning_threshold').value)
        self.battery_critical_threshold = float(self.get_parameter('battery_critical_threshold').value)
        self.zero_rpm_timeout = float(self.get_parameter('zero_rpm_timeout').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)
        self.history_len = int(self.get_parameter('history_length').value)

        # ROS interfaces
        self.status_sub = self.create_subscription(
            RobotStatus, '/robot_status', self._on_robot_status, 10
        )
        self.health_pub = self.create_publisher(String, '/health_status', 10)

        # Health history storage
        self.history = deque(maxlen=self.history_len)
        self.latest_msg: Optional[RobotStatus] = None
        self.zero_rpm_accum = 0.0

        # Publish timer
        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._on_tick)

        # Create service for health history
        self.srv = self.create_service(Trigger, '/get_health_history', self._handle_get_history)

        self.get_logger().info('health_analyzer_node started with configurable thresholds.')

    def _on_robot_status(self, msg: RobotStatus):
        self.latest_msg = msg

    def _on_tick(self):
        if self.latest_msg is None:
            self._publish_health('WARNING', 'No RobotStatus received yet')
            return

        msg = self.latest_msg
        left_rpm = float(msg.left_rpm)
        right_rpm = float(msg.right_rpm)
        battery = float(msg.battery_charge)
        bit_error = (msg.left_bit_error or msg.right_bit_error or msg.battery_bit_error)

        within_rpm = (-self.rpm_limit <= left_rpm <= self.rpm_limit) and \
                     (-self.rpm_limit <= right_rpm <= self.rpm_limit)

        both_zero = abs(left_rpm) < 1e-3 and abs(right_rpm) < 1e-3
        if both_zero:
            self.zero_rpm_accum += 1.0 / self.publish_rate_hz
        else:
            self.zero_rpm_accum = 0.0

        if battery < self.battery_critical_threshold or self.zero_rpm_accum > self.zero_rpm_timeout:
            health = 'CRITICAL'
            reason = f"battery={battery:.1f}% or both motors idle >{self.zero_rpm_timeout}s"
        elif (self.battery_critical_threshold <= battery <= self.battery_warning_threshold) or (not within_rpm) or bit_error:
            health = 'WARNING'
            reason = f"battery={battery:.1f}%, rpm out of range or bit error"
        else:
            health = 'HEALTHY'
            reason = f"OK (L={left_rpm:.0f}, R={right_rpm:.0f}, battery={battery:.1f}%)"

        self._append_history(health)
        self._publish_health(health, reason)

    def _append_history(self, status: str):
        self.history.append(status)

    def _publish_health(self, health: str, reason: str):
        hist = ",".join(list(self.history)[-self.history_len:])
        msg = String()
        msg.data = f"{health} | {reason} | history[{len(self.history)}]: {hist}"
        self.health_pub.publish(msg)
        self.get_logger().info(f"Published to /health_status: {msg.data}")

    def _handle_get_history(self, request, response):
        hist_list = list(self.history)
        response.success = True
        response.message = f"Last {len(hist_list)} readings: {', '.join(hist_list)}"
        self.get_logger().info(f"Service /get_health_history called: {response.message}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HealthAnalyzerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
