import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from datetime import datetime
from typing import Optional, Dict
from interfaces.msg import CanFrame  


class AlertManagerNode(Node):
    """
    ROS2 node that subscribes to /health_status and /can_rx,
    logs alerts and handles emergency stops.
    """

    def __init__(self) -> None:
        """Initialize the Alert Manager node."""
        super().__init__('alert_manager')

        # Health status subscription
        self.subscription = self.create_subscription(
            String,
            '/health_status',
            self.listener_callback,
            10
        )

        # CAN bus subscription
        self.can_subscription = self.create_subscription(
            CanFrame,
            '/can_rx',
            self.can_callback,
            10
        )

        # Internal state
        self.last_status: Optional[str] = None
        self.last_alert_time: Dict[str, Optional[float]] = {
            'HEALTHY': None,
            'WARNING': None,
            'CRITICAL': None
        }
        self.alert_interval: float = 10.0  # seconds

        self.get_logger().info("AlertManagerNode started (subscribed to /health_status and /can_rx)")

    def listener_callback(self, msg: String) -> None:
        """Process incoming health status messages and log accordingly."""
        try:
            current_status = msg.data.split('|')[0].strip() if '|' in msg.data else msg.data.strip()
            now = self.get_clock().now().seconds_nanoseconds()[0]

            if current_status != self.last_status:
                self.get_logger().info(f"State changed: {self.last_status} -> {current_status}")
                self.last_status = current_status

            last_time = self.last_alert_time.get(current_status)
            if last_time and (now - last_time) < self.alert_interval:
                return
            self.last_alert_time[current_status] = now

            if current_status.startswith('HEALTHY'):
                self.get_logger().info("Robot status: HEALTHY")
            elif current_status.startswith('WARNING'):
                self.get_logger().warn(f"Robot Warning: {msg.data}")
            elif current_status.startswith('CRITICAL'):
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                self.get_logger().error(f"CRITICAL ALERT at {timestamp}: {msg.data}")
                print(f"[{timestamp}] CRITICAL ALERT: {msg.data}")
            else:
                self.get_logger().warn(f"Unknown health status: {msg.data}")

        except Exception as e:
            self.get_logger().error(f"Exception in listener_callback: {e}")

    # CAN callback
    def can_callback(self, msg: CanFrame) -> None:
        """Detect and handle emergency stop frames from CAN bus."""
        try:
            if msg.id in (0x103, 0x104) and any(msg.data):
                timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                alert_text = f"EMERGENCY STOP triggered by CAN ID {hex(msg.id)} at {timestamp}"
                self.get_logger().error(alert_text)
                print(alert_text)
        except Exception as e:
            self.get_logger().error(f"Exception in can_callback: {e}")

    def destroy_node(self) -> None:
        """Clean shutdown."""
        self.get_logger().info("Shutting down AlertManagerNode.")
        super().destroy_node()


def main(args=None) -> None:
    """Entry point for the alert_manager node."""
    rclpy.init(args=args)
    node = AlertManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Node interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
