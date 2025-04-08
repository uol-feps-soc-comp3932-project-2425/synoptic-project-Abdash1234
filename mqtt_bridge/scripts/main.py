#!/usr/bin/env python3
#!/usr/bin/env python3

import rospy
import config 
from mqtt_handler import MQTTHandler
from ros_callbacks import battery_callback, odom_callback, scan_callback, imu_callback
import config

# Import ROS message types
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from nav_msgs.msg import Odometry
from movement_controller import MovementController

def main():
    rospy.init_node('mqtt_bridge_node', anonymous=True)

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

    mqtt_handler = MQTTHandler(config.MQTT_BROKER, config.MQTT_PORT, topics, movement_controller=None)
    movement_controller = MovementController(mqtt_handler)
    mqtt_handler.movement_controller = movement_controller

    # Connect to the MQTT broker
    mqtt_handler.connect()
    
    
# ...
# When setting up your subscriber, use a lambda or partial to pass in your mqtt_handler:
    rospy.Subscriber(config.ROS_BATTERY, BatteryState, lambda msg: battery_callback(msg, mqtt_handler))
    rospy.Subscriber(config.ROS_ODOM, Odometry, lambda msg: odom_callback(msg, mqtt_handler))
    rospy.Subscriber(config.ROS_SCAN, LaserScan, lambda msg: scan_callback(msg, mqtt_handler))
    rospy.Subscriber(config.ROS_IMU, Imu, lambda msg: imu_callback(msg, mqtt_handler))
    rospy.Subscriber(config.ROS_ODOM, Odometry, lambda msg: movement_controller.odom_turning_callback(msg))

    print("running bridge")

    def summary_timer_callback(event):
        rospy.loginfo("Summary timer callback triggered")
        mqtt_handler.publish_summary(qos=2)

    rospy.Timer(rospy.Duration(5.0), summary_timer_callback)

    # Keep the node running until it's shut down
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down MQTT Bridge Node")
    finally:
        mqtt_handler.disconnect()

if __name__ == '__main__':
    main()
