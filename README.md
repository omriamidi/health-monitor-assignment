# ROS 2 Health Monitor Assignment

## Assignment Submission for Roboteam

## Overview

This repository contains a complete ROS 2 health monitoring system that classifies robot status into HEALTHY, WARNING, or CRITICAL states. The system integrates with existing robot CAN bus data and provides comprehensive alert management.

## Features Implemented

### Core Requirements
- **Health Analyzer Node**: Subscribes to `/robot_status` and publishes health classifications
- **Alert Manager Node**: Manages alerts with proper logging levels and rate limiting
- **Launch Integration**: Complete launch file with all necessary nodes
- **Health Classification Logic**: 
  HEALTHY: RPMs within range, battery > 25%, no BIT errors
  WARNING: Battery 15-25% OR RPM out of range OR BIT errors
  CRITICAL: Battery < 15% OR both motors idle > 5 seconds

### Bonus Features
- **Configurable Parameters**: All thresholds adjustable via ROS 2 parameters
- **Health History Service**: Service to retrieve last N health status readings
- **Emergency Stop Integration**: CAN frame monitoring for emergency stops (IDs 0x103, 0x104)

## Quick Start

```bash
# Build the package
cd ros2_ws
colcon build --packages-select health_monitor
source install/setup.bash

# Run the system
ros2 launch health_monitor health_monitor.launch.py

# Monitor health status
ros2 topic echo /health_status
```

## Testing

```bash
# Test healthy state
ros2 topic pub -1 /robot_status interfaces/msg/RobotStatus "{left_rpm: 1500, right_rpm: 1500, battery_charge: 90, left_bit_error: 0, right_bit_error: 0, battery_bit_error: 0}"

# Test warning state
ros2 topic pub -1 /robot_status interfaces/msg/RobotStatus "{left_rpm: 2300, right_rpm: 1000, battery_charge: 20, left_bit_error: 0, right_bit_error: 0, battery_bit_error: 0}"

# Test critical state
ros2 topic pub -1 /robot_status interfaces/msg/RobotStatus "{left_rpm: 0, right_rpm: 0, battery_charge: 10, left_bit_error: 1, right_bit_error: 0, battery_bit_error: 0}"
```

## Service Usage

```bash
# Get health history
ros2 service call /get_health_history interfaces/srv/GetHealthHistory "{count: 10}"
```

## Design Approach

1. **Health Analyzer**: Processes robot status data and applies classification logic
2. **Alert Manager**: Handles logging and emergency stop detection
3. **Health History Service**: Provides historical data access

The design emphasizes:
- **Robust error handling** for missing or invalid data
- **Configurable thresholds** for different robot types
- **Rate limiting** to prevent log spam
- **Real-time processing** at 5Hz update rate
- **Integration** with existing CAN bus infrastructure
