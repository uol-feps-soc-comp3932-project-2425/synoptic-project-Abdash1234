#!/usr/bin/env python3
import json
import rospy
import paho.mqtt.client as mqtt
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from nav_msgs.msg import Odometry

# MQTT settings
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_BATTERY = "robot/battery"
MQTT_ODOM    = "robot/odom"
MQTT_SCAN    = "robot/scan"
MQTT_IMU     = "robot/imu"
MQTT_LATENCY = "robot/latency"

# MQTT callback for incoming messages (if needed)
def on_mqtt_message(client, userdata, msg):
    rospy.loginfo("Received MQTT message on topic %s: %s", msg.topic, msg.payload.decode())

# Setup MQTT client
mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
mqtt_client.on_message = on_mqtt_message

# Callback for BatteryState messages
def battery_callback(batt_msg):
    current_time = rospy.Time.now().to_sec()
    original_stamp = batt_msg.header.stamp.to_sec()
    latency = current_time - original_stamp
    rospy.loginfo("Battery Message latency: %.3f seconds", latency)

    data = {
        "header": {
            "frame_id": batt_msg.header.frame_id,
            "stamp": {
                "secs": batt_msg.header.stamp.secs,
                "nsecs": batt_msg.header.stamp.nsecs
            }
        },
        "voltage": batt_msg.voltage,
        "percentage": batt_msg.percentage,
        "present": batt_msg.present
    }
    payload = json.dumps(data)
    returnedLatency = getLatencyData("battery",latency)
    mqtt_client.publish(MQTT_LATENCY, str(returnedLatency))
    mqtt_client.publish(MQTT_BATTERY, payload)

# Callback for Odometry messages
def odom_callback(odom_msg):
    current_time = rospy.Time.now().to_sec()
    original_stamp = odom_msg.header.stamp.to_sec()
    latency = current_time - original_stamp
    rospy.loginfo("Odometry Message latency: %.3f seconds", latency)

    data = {
        "header": {
            "frame_id": odom_msg.header.frame_id,
            "stamp": {
                "secs": odom_msg.header.stamp.secs,
                "nsecs": odom_msg.header.stamp.nsecs
            }
        },
        "position": {
            "x": odom_msg.pose.pose.position.x,
            "y": odom_msg.pose.pose.position.y,
            "z": odom_msg.pose.pose.position.z
        },
        "orientation": {
            "x": odom_msg.pose.pose.orientation.x,
            "y": odom_msg.pose.pose.orientation.y,
            "z": odom_msg.pose.pose.orientation.z,
            "w": odom_msg.pose.pose.orientation.w
        },
        "linear_velocity": odom_msg.twist.twist.linear.x,
        "angular_velocity": odom_msg.twist.twist.angular.z
    }
    payload = json.dumps(data)
    returnedLatency = getLatencyData("odom",latency)
    mqtt_client.publish(MQTT_LATENCY, str(returnedLatency))
    mqtt_client.publish(MQTT_ODOM, payload)

# Callback for LaserScan messages
def scan_callback(scan_msg):
    current_time = rospy.Time.now().to_sec()
    original_stamp = scan_msg.header.stamp.to_sec()
    latency = current_time - original_stamp
    rospy.loginfo("LaserScan Message latency: %.3f seconds", latency)

    data = {
        "header": {
            "frame_id": scan_msg.header.frame_id,
            "stamp": {
                "secs": scan_msg.header.stamp.secs,
                "nsecs": scan_msg.header.stamp.nsecs
            }
        },
        "angle_min": scan_msg.angle_min,
        "angle_max": scan_msg.angle_max,
        "angle_increment": scan_msg.angle_increment,
        "ranges": list(scan_msg.ranges)
    }
    payload = json.dumps(data)

    returnedLatency = getLatencyData("scan",latency)
    mqtt_client.publish(MQTT_LATENCY, str(returnedLatency))
    mqtt_client.publish(MQTT_SCAN, payload)

# Callback for IMU messages
def imu_callback(imu_msg):
    current_time = rospy.Time.now().to_sec()
    original_stamp = imu_msg.header.stamp.to_sec()
    latency = current_time - original_stamp
    rospy.loginfo("IMU Message latency: %.3f seconds", latency)

    data = {
        "header": {
            "frame_id": imu_msg.header.frame_id,
            "stamp": {
                "secs": imu_msg.header.stamp.secs,
                "nsecs": imu_msg.header.stamp.nsecs
            }
        },
        "linear_acceleration": {
            "x": imu_msg.linear_acceleration.x,
            "y": imu_msg.linear_acceleration.y,
            "z": imu_msg.linear_acceleration.z
        },
        "angular_velocity": {
            "x": imu_msg.angular_velocity.x,
            "y": imu_msg.angular_velocity.y,
            "z": imu_msg.angular_velocity.z
        },
        "orientation": {
            "x": imu_msg.orientation.x,
            "y": imu_msg.orientation.y,
            "z": imu_msg.orientation.z,
            "w": imu_msg.orientation.w
        }
    }
    payload = json.dumps(data)
    
    returnedLatency = getLatencyData("imu",latency)
    mqtt_client.publish(MQTT_LATENCY, str(returnedLatency))
    mqtt_client.publish(MQTT_IMU, payload)

def getLatencyData(source,latency):
        latency_data = {
        "source": source,
        "latency": latency
    }
        return latency_data


def mqtt_bridge_node():
    rospy.init_node('mqtt_bridge', anonymous=True)
    # Subscribe to each ROS topic with its corresponding callback
    rospy.Subscriber("/battery", BatteryState, battery_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.Subscriber("/imu", Imu, imu_callback)
    
    rospy.loginfo("MQTT Bridge node started. Bridging ROS topics to MQTT topics.")
    mqtt_client.loop_start()
    rospy.spin()
    mqtt_client.loop_stop()

if __name__ == '__main__':
    try:
        mqtt_bridge_node()
    except rospy.ROSInterruptException:
        pass
