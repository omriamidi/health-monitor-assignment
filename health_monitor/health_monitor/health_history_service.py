import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from collections import deque
from interfaces.srv import GetHealthHistory


class HealthHistoryService(Node):
    def __init__(self):
        super().__init__('health_history_service')

        # Store all health status messages received
        self.history = deque(maxlen=1000)

        # Subscribe to /health_status topic to collect real-time data
        self.subscriber = self.create_subscription(
            String,
            '/health_status',
            self._on_health_status,
            10
        )

        # Set up the service using the new interfaces/srv/GetHealthHistory type
        self.srv = self.create_service(GetHealthHistory, 'get_health_history', self.get_health_history_callback)

        self.get_logger().info("HealthHistoryService started and listening on /get_health_history")

    def _on_health_status(self, msg: String):
        """Store every new health status message received from health_analyzer"""
        self.history.append(msg.data)

    def get_health_history_callback(self, request, response):
        """Return the last N health status messages as requested by the client"""
        n = request.count  # N that the user requested
        n = min(n, len(self.history))  # If there are fewer than full history
        last_n = list(self.history)[-n:] if n > 0 else []

        response.success = True
        response.message = f"Last {n} readings:\n" + "\n".join(last_n)
        self.get_logger().info(f"Returned last {n} readings.")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = HealthHistoryService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
