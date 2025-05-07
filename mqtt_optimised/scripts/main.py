#!/usr/bin/env python3


import rospy
import config 
from mqtt_handler import MQTTHandler
from ros_callbacks import battery_callback, odom_callback, scan_callback, imu_callback
from qos_manager import QoSManager
from aggregator import Aggregator
# from qos_manager import QoSManager
import config, threading
from event_manager import EventManager
import events

# Import ROS message types
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from nav_msgs.msg import Odometry
from movement_controller import MovementController
from metrics_manager import MetricsManager



def main():
    rospy.init_node('mqtt_bridge_node', anonymous=True)

    # Define MQTT topics to subscribe and publish to
    topics = {
    "subscribe": [config.MQTT_RAW_COMMAND],
    "publish": {
        "battery": config.MQTT_BATTERY,
        "odom": config.MQTT_ODOM,
        "scan": config.MQTT_SCAN,
        "imu": config.MQTT_IMU,
        "latency": config.MQTT_LATENCY,
        "throughput": config.MQTT_THROUGHPUT,
        "data": config.MQTT_DATA
    }
}
    # Initialise metrics manager and start timers for measuring latency, throughput, etc.

    metrics = MetricsManager()
    metrics.start_timers()

    # Start QoS manager in a background thread
    qos_manager = QoSManager(metrics)
    qos_thread = threading.Thread(target=qos_manager.start,daemon=True)
    qos_thread.start()
    
    # Register event handlers (e.g., for low battery or high latency)
    event_manager = EventManager()
    event_manager.register(events.LOW_BATTERY_EVENT, events.on_low_battery)
    event_manager.register(events.HIGH_LATENCY_EVENT, events.on_high_latency)

    # Create MQTT handler and movement controller
    mqtt_handler = MQTTHandler(config.MQTT_BROKER, config.MQTT_PORT, topics, movement_controller=None,metrics=metrics,qos = qos_manager)
    movement_controller = MovementController(mqtt_handler)
    mqtt_handler.movement_controller = movement_controller

    # Aggregator for batching battery messages before publishing
    battery_aggregator = Aggregator(mqtt_handler, config.MQTT_BATTERY, flush_interval=6.0)    
    
    # Connect to the MQTT broker
    mqtt_handler.connect()
    
    # Set up ROS subscribers with callbacks that also reference the MQTT handler and other managers
    rospy.Subscriber(config.ROS_BATTERY, BatteryState, lambda msg: battery_callback(msg, mqtt_handler, metrics, aggregator = battery_aggregator, event = event_manager, qos_manager = qos_manager))
    rospy.Subscriber(config.ROS_ODOM, Odometry, lambda msg: odom_callback(msg, mqtt_handler, metrics, event = event_manager, qos_manager = qos_manager))
    rospy.Subscriber(config.ROS_SCAN, LaserScan, lambda msg: scan_callback(msg, mqtt_handler, metrics, event = event_manager, qos_manager = qos_manager))
    rospy.Subscriber(config.ROS_IMU, Imu, lambda msg: imu_callback(msg, mqtt_handler, metrics, event = event_manager, qos_manager = qos_manager))
    rospy.Subscriber(config.ROS_ODOM, Odometry, lambda msg: movement_controller.odom_turning_callback(msg))

    # Periodically publish system summary via MQTT every 5 seconds

    def summary_timer_callback(event):
        rospy.loginfo("Summary timer callback triggered")
        mqtt_handler.publish_summary(qos=2)

    rospy.Timer(rospy.Duration(5.0), summary_timer_callback)

    # Register shutdown cleanup to stop the aggregator cleanly
    rospy.on_shutdown(lambda: shutdown_cleanup(battery_aggregator))

    # Keep the node running until it's shut down
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down MQTT Bridge Node")
    finally:
        mqtt_handler.disconnect()




def shutdown_cleanup(battery_aggregator):
    rospy.loginfo("Shutting down: stopping aggregator timer")
    battery_aggregator.stop()

if __name__ == '__main__':
    main()