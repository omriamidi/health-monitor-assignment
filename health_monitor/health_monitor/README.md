# Health Monitor Package

## Overview
A ROS 2 package that monitors robot health by subscribing to `/robot_status` and classifying the system into HEALTHY, WARNING, or CRITICAL states. It publishes to `/health_status`, manages alerts with throttling, and integrates with CAN emergency stop messages.

## Nodes
- **health_analyzer_node**: Analyzes robot status and publishes health classifications
- **alert_manager_node**: Manages alerts and emergency stop detection
- **health_history_service**: Provides service to retrieve health history

## Build
```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```


## Run

**Terminal 1:**
```bash
ros2 run ros2can can_ros2_status_node
```

**Terminal 2:**
```bash
ros2 launch health_monitor health_monitor.launch.py
```

**Terminal 3:**
```bash
ros2 topic echo /health_status
```


## Design
The Health Analyzer Node classifies the robot's condition based on motor RPM ranges, battery charge levels, and BIT error flags. The node publishes updates at 5 Hz and maintains a rolling window of the last 10 readings to smooth transient fluctuations. The Alert Manager listens for health status changes and logs alerts with rate limiting (10-second cooldown per alert type).


## Parameters
- `rpm_limit` (float, default 2000.0)
- `battery_warning_threshold` (float, default 25.0)
- `battery_critical_threshold` (float, default 15.0)
- `zero_rpm_timeout` (float, default 5.0)
- `publish_rate_hz` (float, default 5.0)
- `history_length` (int, default 10)


## Example Tests

**Healthy:**
```bash
ros2 topic pub -1 /robot_status interfaces/msg/RobotStatus "{left_rpm: 1500, right_rpm: 1500, battery_charge: 90, left_bit_error: 0, right_bit_error: 0, battery_bit_error: 0}"
```

**Warning:**
```bash
ros2 topic pub -1 /robot_status interfaces/msg/RobotStatus "{left_rpm: 2300, right_rpm: 1000, battery_charge: 20, left_bit_error: 0, right_bit_error: 0, battery_bit_error: 0}"
```

**Critical:**
```bash
ros2 topic pub -1 /robot_status interfaces/msg/RobotStatus "{left_rpm: 0, right_rpm: 0, battery_charge: 10, left_bit_error: 1, right_bit_error: 0, battery_bit_error: 0}"
```


## Bonus Features
- ROS 2 parameters for configurable thresholds.
- Service returning N most recent health statuses.
- Emergency stop alerts from CAN IDs 0x103 / 0x104.


## Example Service Call
To fetch the last N health readings:
```bash
ros2 service call /get_health_history interfaces/srv/GetHealthHistory "{count: 10}"
```

