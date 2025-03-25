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
MQTT_THROUGHPUT = "robot/throughput"
MQTT_DATA = "robot/data"

# Global throughput counters (messages counted per topic)
throughput_counters = {
    "battery": 0,
    "odom": 0,
    "scan": 0,
    "imu": 0,
}

# MQTT callback for incoming messages (if needed)
def on_mqtt_message(client, userdata, msg):
    rospy.loginfo("Received MQTT message on topic %s: %s", msg.topic, msg.payload.decode())

# Setup MQTT client
mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
mqtt_client.on_message = on_mqtt_message

# Callback for BatteryState messages
def battery_callback(batt_msg):
    
    global throughput_counters
    throughput_counters["battery"] += 1

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
    helper("battery",payload,latency,MQTT_BATTERY)

# Callback for Odometry messages
def odom_callback(odom_msg):
        
    global throughput_counters
    throughput_counters["odom"] += 1

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
    helper("odom",payload,latency,MQTT_ODOM)

# Callback for LaserScan messages
def scan_callback(scan_msg):
        
    global throughput_counters
    throughput_counters["scan"] += 1

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
    helper("scan",payload,latency,MQTT_SCAN)

# Callback for IMU messages
def imu_callback(imu_msg):
        
    global throughput_counters
    throughput_counters["imu"] += 1

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
    helper("imu",payload,latency,MQTT_IMU)

def sendLatencyData(source,latency):
        latency_data = {
        "source": source,
        "latency": latency
    }
        
        mqtt_client.publish(MQTT_LATENCY, str(latency_data))

def sendBandwidth(source, payload):

    payload_bytes = payload.encode('utf-8')  # Ensure it's in bytes (UTF-8 is standard for JSON)
    payload_size = len(payload_bytes)

    bandwidth_data ={
        "source": source,
        "payload_size": payload_size
    }

    mqtt_client.publish(MQTT_DATA, str(bandwidth_data))

def helper(source,payload,latency,msg):
    
    sendBandwidth(source,payload)
    sendLatencyData(source,latency)
    mqtt_client.publish(msg, payload)



# Timer callback to log and reset throughput counters
def log_throughput(event):
    global throughput_counters
    interval = event.current_real.to_sec() - event.last_real.to_sec() if event.last_real else 1.0
    battery_throughput = throughput_counters["battery"] / interval
    odom_throughput = throughput_counters["odom"] / interval
    scan_throughput = throughput_counters["scan"] / interval
    imu_throughput = throughput_counters["imu"] / interval

    throughput_data = {
        "battery": battery_throughput,
        "odom": odom_throughput,
        "scan": scan_throughput,
        "imu": imu_throughput
    }
    rospy.loginfo("Throughput (msg/sec): %s", throughput_data)
    # Optionally publish throughput metrics to MQTT
    mqtt_client.publish(MQTT_THROUGHPUT, json.dumps(throughput_data))
    
    # Reset counters
    throughput_counters["battery"] = 0
    throughput_counters["odom"] = 0
    throughput_counters["scan"] = 0
    throughput_counters["imu"] = 0


def mqtt_bridge_node():
    rospy.init_node('mqtt_bridge', anonymous=True)
    # Subscribe to each ROS topic with its corresponding callback
    rospy.Subscriber("/battery", BatteryState, battery_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.Subscriber("/imu", Imu, imu_callback)

    rospy.Timer(rospy.Duration(5.0), log_throughput)
    
    rospy.loginfo("MQTT Bridge node started. Bridging ROS topics to MQTT topics.")
    mqtt_client.loop_start()
    rospy.spin()
    mqtt_client.loop_stop()

if __name__ == '__main__':
    try:
        mqtt_bridge_node()
    except rospy.ROSInterruptException:
        pass
