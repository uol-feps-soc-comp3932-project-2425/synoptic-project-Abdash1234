# ros_callbacks.py

import json
import config  # Contains MQTT_BATTERY, etc.
from sensor_msgs.msg import BatteryState, LaserScan, Imu
from nav_msgs.msg import Odometry

def battery_callback(batt_msg, mqtt_handler):
    # Prepare the battery data payload as a dictionary
    battery_data = {
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

    # Serialize the battery data using CBOR
    payload = json.dumps(battery_data)
    mqtt_handler.publish(config.MQTT_BATTERY, payload)

def odom_callback(odom_msg, mqtt_handler):
    odom_data = {
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
    
    payload = json.dumps(odom_data)
    mqtt_handler.publish(config.MQTT_ODOM, payload)

def scan_callback(scan_msg, mqtt_handler):
    
    scan_data = {
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
    payload = json.dumps(scan_data)
    mqtt_handler.publish(config.MQTT_SCAN, payload)

def imu_callback(imu_msg, mqtt_handler):

    imu_data = {
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
    payload = json.dumps(imu_data)
    mqtt_handler.publish(config.MQTT_IMU, payload)

