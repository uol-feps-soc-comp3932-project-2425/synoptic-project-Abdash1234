#!/usr/bin/env python3
import json, rospy
import paho.mqtt.client as mqtt
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from nav_msgs.msg import Odometry
import statistics
import cbor2

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

latest_battery_percentage = None
overall_mean_latency = None
overall_processing_stdev = None
overall_error_rate = None
overall_bandwidth_usage = None


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
    global throughput_counters, latest_battery_percentage
    throughput_counters["battery"] += 1

    current_time = rospy.Time.now().to_sec()
    original_stamp = batt_msg.header.stamp.to_sec()
    latency = current_time - original_stamp
    latency_records["battery"].append(latency)

    latest_battery_percentage = batt_msg.percentage

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
    # payload = json.dumps(data)
    payload = cbor2.dumps(data)
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

    # payload = json.dumps(data)
    payload = cbor2.dumps(data)
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
    # payload = json.dumps(data)
    payload = cbor2.dumps(data)
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
    # payload = json.dumps(data)
    payload = cbor2.dumps(data)
    helper("imu",payload,latency,MQTT_IMU)

    finish_time = rospy.Time.now().to_sec()
    calcProcessingTime(start_time, finish_time, "imu")

def getBatteryLevel():
    global latest_battery_percentage
    if latest_battery_percentage != None:
        return latest_battery_percentage
    else:
        rospy.logwarn("Battery level not available")
        return -1
    
def getCurrentAverageLatency():
    return overall_mean_latency

def getBandwidth():
    return bandwidth_counters

def ErrorRate():    
    return overall_error_rate

# Function to set QoS level
def setDynamicQoS(source):
    if source == "battery":
        return 0
    elif source == "odom":
        return 0
    elif source == "scan":
        return 0
    elif source == "imu":
        return 0
    else:
        return 0

# Function to publish message
def mqttPublish(destination, msg, source=False):
    try:
        mqtt_client.publish(destination, msg)
    except Exception as e:
        rospy.logerr("Error publishing to topic %s: %s", destination, e)
        if source and source in error_counters:
            error_counters[source] += 1
        else:
            rospy.logwarn("No valid source provided for error counting")

# Helper functions
def sendLatencyData(source,latency):
        latency_data = {
        "source": source,
        "latency": latency
    }
        try:
            # mqttPublish(MQTT_LATENCY, json.dumps(latency_data), source)
            mqttPublish(MQTT_LATENCY, cbor2.dumps(latency_data), source)
        except Exception as e:
            rospy.logerr("Error publishing latency for %s: %s", source, e)
            error_counters[source] += 1
    
def sendBandwidth(source, payload):

    # payload_bytes = payload.encode('utf-8')  # Ensure it's in bytes (UTF-8 is standard for JSON)
    # payload_size = len(payload_bytes)
    payload_size = len(payload)  # Assuming payload is already in bytes

    bandwidth_counters[source] += payload_size

    bandwidth_data ={
        "source": source,
        "payload_size": payload_size
    }

    try:
        # mqttPublish(MQTT_DATA, json.dumps(bandwidth_data), source)
            mqttPublish(MQTT_DATA, cbor2.dumps(bandwidth_data), source)

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
        mqttPublish(msg, payload, source)
    except Exception as e:
        rospy.logerr("Error publishing to topic %s for %s: %s", msg, source, e)
        error_counters[source] += 1

def write_metrics_to_file(metrics):
    with open("/home/abdullah/catkin_ws/src/synoptic-project-Abdash1234/mqtt_optimised/scripts/metrics.txt", "a") as f:
        # Convert the metrics dictionary to a formatted JSON string
        metrics_str = json.dumps(metrics, indent=2)
        # Write the metrics with a newline separator
        f.write(metrics_str + "\n")

def log_metrics(event):
    global throughput_counters, latency_records, error_counters, processing_records,overall_mean_latency, overall_processing_stdev, overall_error_rate, overall_bandwidth_usage
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
    all_latencies = []
    for topic in latency_records:
        all_latencies.extend(latency_records[topic])
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

    if all_latencies:
        overall_mean_latency = statistics.mean(all_latencies)
    else:
        overall_mean_latency = 0.0

    # Calculate processing overhead stats for each topic
    processing_stats = {}
    all_processing = []
    for topic in processing_records:
        # Make a local copy of the data to avoid concurrent modifications
        local_data = list(processing_records[topic])
        # Aggregate all processing times across topics
        all_processing.extend(local_data)
        
        if local_data:
            mean_processing = statistics.mean(local_data)
            stdev_processing = statistics.stdev(local_data) if len(local_data) > 1 else 0.0
            # Here, jitter is defined as the range.
            processing_jitter = max(local_data) - min(local_data)
        else:
            mean_processing = stdev_processing = processing_jitter = 0.0
            
        processing_stats[topic + "_processing_mean"] = mean_processing
        processing_stats[topic + "_processing_stdev"] = stdev_processing
        processing_stats[topic + "_processing_jitter"] = processing_jitter
        
        processing_records[topic].clear()  # Clear the original list for the next interval

    # Compute overall standard deviation for all processing times (as a measure of overall jitter)
    if len(all_processing) > 1:
        overall_processing_stdev = statistics.stdev(all_processing)
    else:
        overall_processing_stdev = 0.0

    # (Bandwidth is being reported per message in sendBandwidth)
    # Calculate error percentages for each topic
    error_percentages = {}
    overall_errors = 0
    overall_attempts = 0
    for topic in error_counters:
        topic_attempts = throughput_counters[topic] + error_counters[topic]
        if topic_attempts > 0:
            error_percentages[topic + "_error_pct"] = (error_counters[topic] / topic_attempts) * 100
        else:
            error_percentages[topic + "_error_pct"] = 0.0
        overall_errors += error_counters[topic]
        overall_attempts += topic_attempts

    overall_error_rate = (overall_errors / overall_attempts) * 100 if overall_attempts > 0 else 0.0

    # Calculate bandwidth usage per topic and overall (in bytes per second)
    overall_bytes = sum(bandwidth_counters.values())
    overall_bandwidth_usage = overall_bytes / interval  


    metrics = {
        "throughput": throughput_data,
        "latency": latency_stats,
        "processing": processing_stats,
        "errors": error_percentages,
    }

    rospy.loginfo("Average Latency: %.3f ms", overall_mean_latency*1000)
    rospy.loginfo("Current Battery Levels : %.3f", getBatteryLevel()*100)
    rospy.loginfo("Overall Processing Jitter (stdev): %fms", overall_processing_stdev)
    rospy.loginfo("Overall Bandwidth Usage: %f bytes/s", (overall_bandwidth_usage/5))
    rospy.loginfo("Overall Error Rate: %.3f%%", overall_error_rate)
    rospy.loginfo("Metrics: %s", metrics)
    # mqttPublish(MQTT_THROUGHPUT, json.dumps(metrics))
    mqttPublish(MQTT_THROUGHPUT, cbor2.dumps(metrics))
    write_metrics_to_file(metrics)
    
    # Reset counters for next interval
    throughput_counters["battery"] = 0
    throughput_counters["odom"] = 0
    throughput_counters["scan"] = 0
    throughput_counters["imu"] = 0

    error_counters["battery"] = 0
    error_counters["odom"] = 0
    error_counters["scan"] = 0
    error_counters["imu"] = 0

def mqtt_bridge_node():
    rospy.init_node('mqtt_bridge', anonymous=True)
    # Subscribe to each ROS topic with its corresponding callback
    rospy.Subscriber("/battery", BatteryState, battery_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.Subscriber("/imu", Imu, imu_callback)

    rospy.Timer(rospy.Duration(5.0), log_metrics)
    
    rospy.loginfo("MQTT Bridge node started. Bridging ROS topics to MQTT topics.")
    mqtt_client.loop_start()
    rospy.spin()
    mqtt_client.loop_stop()

if __name__ == '__main__':
    try:
        mqtt_bridge_node()
    except rospy.ROSInterruptException:
        pass
