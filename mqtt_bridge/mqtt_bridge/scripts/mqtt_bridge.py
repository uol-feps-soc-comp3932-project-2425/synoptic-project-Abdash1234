#!/usr/bin/env python3
import json
import rospy
import paho.mqtt.client as mqtt
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from nav_msgs.msg import Odometry
import statistics

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

# Global latency records (latency recorded per topic)
latency_records = {
    "battery": [],
    "odom": [],
    "scan": [],
    "imu": []
}

# Global error counters for each topic
error_counters = {
    "battery": 0,
    "odom": 0,
    "scan": 0,
    "imu": 0,
}

# Global bandwidth counters (total bytes transmitted per topic)
bandwidth_counters = {
    "battery": 0,
    "odom": 0,
    "scan": 0,
    "imu": 0,
}

# Global processing overhead records (processing time per callback for each topic)
processing_records = {
    "battery": [],
    "odom": [],
    "scan": [],
    "imu": []
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
    
    start_time = rospy.Time.now().to_sec()
    global throughput_counters
    throughput_counters["battery"] += 1

    current_time = rospy.Time.now().to_sec()
    original_stamp = batt_msg.header.stamp.to_sec()
    latency = current_time - original_stamp
    latency_records["battery"].append(latency)

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
    finish_time = rospy.Time.now().to_sec()
    calcProcessingTime(start_time, finish_time, "battery")

# Callback for Odometry messages
def odom_callback(odom_msg):

    start_time = rospy.Time.now().to_sec()

    global throughput_counters
    throughput_counters["odom"] += 1

    current_time = rospy.Time.now().to_sec()
    original_stamp = odom_msg.header.stamp.to_sec()
    latency = current_time - original_stamp
    latency_records["odom"].append(latency)

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

    finish_time = rospy.Time.now().to_sec()
    calcProcessingTime(start_time, finish_time, "odom")

# Callback for LaserScan messages
def scan_callback(scan_msg):
    
    start_time = rospy.Time.now().to_sec()

    global throughput_counters
    throughput_counters["scan"] += 1

    current_time = rospy.Time.now().to_sec()
    original_stamp = scan_msg.header.stamp.to_sec()
    latency = current_time - original_stamp
    latency_records["scan"].append(latency)

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

    finish_time = rospy.Time.now().to_sec()
    calcProcessingTime(start_time, finish_time, "scan")

# Callback for IMU messages
def imu_callback(imu_msg):
        
    start_time = rospy.Time.now().to_sec()
 
    global throughput_counters
    throughput_counters["imu"] += 1

    current_time = rospy.Time.now().to_sec()
    original_stamp = imu_msg.header.stamp.to_sec()
    latency = current_time - original_stamp
    # rospy.loginfo("IMU Message latency: %.3f seconds", latency)
    latency_records["imu"].append(latency)

    
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

    finish_time = rospy.Time.now().to_sec()
    calcProcessingTime(start_time, finish_time, "scan")

# Helper functions
def sendLatencyData(source,latency):
        latency_data = {
        "source": source,
        "latency": latency
    }
        try:
            mqtt_client.publish(MQTT_LATENCY, json.dumps(latency_data))
        except Exception as e:
            rospy.logerr("Error publishing latency for %s: %s", source, e)
            error_counters[source] += 1
    
def sendBandwidth(source, payload):

    payload_bytes = payload.encode('utf-8')  # Ensure it's in bytes (UTF-8 is standard for JSON)
    payload_size = len(payload_bytes)

    bandwidth_counters[source] += payload_size

    bandwidth_data ={
        "source": source,
        "payload_size": payload_size
    }

    try:
        mqtt_client.publish(MQTT_DATA, json.dumps(bandwidth_data))
    except Exception as e:
        rospy.logerr("Error publishing bandwidth for %s: %s", source, e)
        error_counters[source] += 1

def calcProcessingTime(startTime, endTime, source):
    processing_time = endTime - startTime
    processing_records[source].append(processing_time)


def helper(source,payload,latency,msg):
    
    sendBandwidth(source,payload)
    sendLatencyData(source,latency)
    try:
        mqtt_client.publish(msg, payload)
    except Exception as e:
        rospy.logerr("Error publishing to topic %s for %s: %s", msg, source, e)
        error_counters[source] += 1

def log_throughput(event):
    global throughput_counters, latency_records, error_counters, processing_records
    interval = event.current_real.to_sec() - (event.last_real.to_sec() if event.last_real else event.current_real.to_sec() - 1.0)
    battery_throughput = throughput_counters["battery"] / interval
    odom_throughput = throughput_counters["odom"] / interval
    scan_throughput = throughput_counters["scan"] / interval
    imu_throughput = throughput_counters["imu"] / interval

    throughput_data = {
        "battery_msg_per_sec": battery_throughput,
        "odom_msg_per_sec": odom_throughput,
        "scan_msg_per_sec": scan_throughput,
        "imu_msg_per_sec": imu_throughput,
    }

    # Calculate latency stats for each topic
    latency_stats = {}
    for topic in latency_records:
        if latency_records[topic]:
            mean_latency = statistics.mean(latency_records[topic])
            stdev_latency = statistics.stdev(latency_records[topic]) if len(latency_records[topic]) > 1 else 0.0
            jitter = max(latency_records[topic]) - min(latency_records[topic])
        else:
            mean_latency = stdev_latency = jitter = 0.0
        latency_stats[topic + "_latency_mean"] = mean_latency
        latency_stats[topic + "_latency_stdev"] = stdev_latency
        latency_stats[topic + "_jitter"] = jitter
        latency_records[topic].clear()  # Reset for next interval

    # Calculate processing overhead stats for each topic
    processing_stats = {}
    for topic in processing_records:
        if processing_records[topic]:
            mean_processing = statistics.mean(processing_records[topic])
            stdev_processing = statistics.stdev(processing_records[topic]) if len(processing_records[topic]) > 1 else 0.0
            processing_jitter = max(processing_records[topic]) - min(processing_records[topic])
        else:
            mean_processing = stdev_processing = processing_jitter = 0.0
        processing_stats[topic + "_processing_mean"] = mean_processing
        processing_stats[topic + "_processing_stdev"] = stdev_processing
        processing_stats[topic + "_processing_jitter"] = processing_jitter
        processing_records[topic].clear()

    # (Bandwidth is being reported per message in sendBandwidth)
    # Calculate error percentages for each topic
    error_percentages = {}
    for topic in error_counters:
        total_attempts = throughput_counters[topic] + error_counters[topic]
        if total_attempts > 0:
            error_percentages[topic + "_error_pct"] = (error_counters[topic] / total_attempts) * 100
        else:
            error_percentages[topic + "_error_pct"] = 0.0

    metrics = {
        "throughput": throughput_data,
        "latency": latency_stats,
        "processing": processing_stats,
        "errors": error_percentages,
    }
    rospy.loginfo("Metrics: %s", metrics)
    mqtt_client.publish(MQTT_THROUGHPUT, json.dumps(metrics))
    
    # Reset counters for next interval
    throughput_counters["battery"] = 0
    throughput_counters["odom"] = 0
    throughput_counters["scan"] = 0
    throughput_counters["imu"] = 0

    error_counters["battery"] = 0
    error_counters["odom"] = 0
    error_counters["scan"] = 0
    error_counters["imu"] = 0
    error_counters["imu"] = 0



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
