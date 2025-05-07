# ROS-MQTT Communication Optimisation for Mobile Robotics

This project implements a modular system to optimise MQTT-based communication between a ROS-enabled mobile robot (e-puck 2) and external clients. It includes application-level enhancements such as adaptive QoS, message aggregation, event-driven messaging, and CBOR-based data compression. These optimisations improve latency, reliability, and bandwidth efficiency in constrained, real-time environments.

## Features

- **Adaptive QoS**: Dynamically adjusts message reliability based on network conditions.
- **Event-Driven Messaging**: Triggers overrides (e.g., low battery, packet loss) for critical events.
- **Message Aggregation**: Batches redundant sensor data to reduce overhead.
- **CBOR Compression**: Replaces JSON with compact binary encoding for lower payload size.
- **ROS-MQTT Bridge**: Bi-directional communication between ROS and MQTT topics.

## System Overview

- Robot: [e-puck 2](https://www.gctronic.com/e-puck2.php)
- Middleware: ROS Noetic
- Communication Protocol: MQTT (via Mosquitto broker)
- Language: Python 3

## Structure

    .
    ├── main.py                # Entry point and coordinator
    ├── mqtt_handler.py        # MQTT connection logic
    ├── qos_manager.py         # Adaptive QoS logic
    ├── aggregator.py          # Message batching logic
    ├── event_manager.py       # Event-based override system
    ├── ros_callbacks.py       # ROS topic subscriptions
    ├── movement_controller.py # Robot movement control
    ├── config.py              # Global config/thresholds



## Getting Started

1. Install dependencies:
   ```bash
   pip install paho-mqtt cbor2 rospy
   roslaunch epuck_driver epuck_ros2.launch
   python main.py
  (Note: Requires an MQTT broker (e.g., Mosquitto) running locally or remotely)
