#!/usr/bin/env python3


import rospy
import config 
from mqtt_handler import MQTTHandler
from ros_callbacks import battery_callback, odom_callback, scan_callback, imu_callback
# from qos_manager import QoSManager
import config

# # Import ROS message types
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

     # Step 1: Instantiate the MQTTHandler without a movement controller.
    mqtt_handler = MQTTHandler(config.MQTT_BROKER, config.MQTT_PORT, topics, movement_controller=None)
    
    # Step 2: Instantiate the MovementController, passing the mqtt_handler.
    movement_controller = MovementController(mqtt_handler)
    
    # Step 3: Inject the movement controller into the MQTTHandler.
    mqtt_handler.movement_controller = movement_controller
    
    # Connect to the MQTT broker
    mqtt_handler.connect()
    
    
# ...
# When setting up your subscriber, use a lambda or partial to pass in your mqtt_handler:
    rospy.Subscriber(config.ROS_BATTERY, BatteryState, lambda msg: battery_callback(msg, mqtt_handler))
    rospy.Subscriber(config.ROS_ODOM, Odometry, lambda msg: odom_callback(msg, mqtt_handler))
    rospy.Subscriber(config.ROS_SCAN, LaserScan, lambda msg: scan_callback(msg, mqtt_handler))
    rospy.Subscriber(config.ROS_IMU, Imu, lambda msg: imu_callback(msg, mqtt_handler))

    
    

    # rospy.Subscriber("/odom", Odometry, odom_callback)
    # rospy.Subscriber("/scan", LaserScan, scan_callback)
    # rospy.Subscriber("/imu", Imu, imu_callback)
    # rospy.Subscriber("/odom", Odometry, odom_turning_callback)
    
    # mqtt_handler.publish(topics["publish"]["battery"], battery_payload)


    # If you have ROS callbacks that need to publish MQTT messages,
    # you can pass the mqtt_handler instance to them.
    # For example, if you set up your ROS subscribers in another module,
    # you can provide the mqtt_handler to the callback functions.

    # Keep the node running until it's shut down
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down MQTT Bridge Node")
    finally:
        mqtt_handler.disconnect()

if __name__ == '__main__':
    main()