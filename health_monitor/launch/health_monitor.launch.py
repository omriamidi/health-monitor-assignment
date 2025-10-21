from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for Roboteam Health Monitor System.
    Runs:
      1. can_ros2_status_node  - provides robot CAN data (simulated locally)
      2. health_analyzer_node  - processes health logic
      3. alert_manager_node    - handles logging and alerts
    """

    return LaunchDescription([
        # Simulated CAN node (for local testing)
        Node(
            package='ros2can',  # in real system this provides data
            executable='can_ros2_status_node',
            name='can_status_node',
            output='screen',
            emulate_tty=True  # ensures colored logs
        ),

        # Health Analyzer node
        Node(
            package='health_monitor',
            executable='health_analyzer_node',
            name='health_analyzer',
            output='screen',
            emulate_tty=True
        ),

        # Alert Manager node
        Node(
            package='health_monitor',
            executable='alert_manager',
            name='alert_manager',
            output='screen',
            emulate_tty=True
        ),
    ])
