#!/usr/bin/env python3
import json, rospy
import time
import paho.mqtt.client as mqtt
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from nav_msgs.msg import Odometry
import statistics, hashlib, psutil, math, tf, cbor2
from geometry_msgs.msg import Twist 

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
MQTT_OPTIMISED_COMMAND = "optimised/command"
MQTT_RAW_COMMAND = "raw/command"


latest_battery_percentage = None
overall_mean_latency = None
overall_processing_stdev = None
overall_error_rate = None
overall_bandwidth_usage = None

turning = False
initial_theta = None
current_turn_threshold = None  # In radians


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

# Global dictionary to store publish timestamps
publish_timestamps = {}
mqtt_publish_latencies = []  # To store latencies for statistics
mqtt_errors = 0
mqtt_total_bytes = 0
mqtt_overall_bytes = 0
duplicate_count = 0
message_hash_counts = {}

command_seq = 0

# MQTT callback for incoming messages (if needed)
def on_mqtt_message(client, userdata, msg):
    if msg.topic == MQTT_RAW_COMMAND:
        readMessage(msg)
    # Compute hash for the incoming message payload
    global duplicate_count
    payload_hash = hashlib.sha256(msg.payload).hexdigest()
    # Increment the count for this payload hash
    if payload_hash in message_hash_counts:
        message_hash_counts[payload_hash] += 1
        # Every message beyond the first is considered a duplicate
        duplicate_count += 1
    else:
        message_hash_counts[payload_hash] = 1
    rospy.loginfo("Received MQTT message on topic %s: %s", msg.topic, msg.payload.decode())

def readMessage(msg):
    global mqtt_client, turning, initial_theta, current_turn_threshold, cmd_pub, command_seq
    print("in read message")
    try:
        # Decode the incoming JSON message
        command_data = json.loads(msg.payload.decode('utf-8'))
        rospy.loginfo("Received raw command: %s", command_data)
        
        # Read the speed from the command data; default to 2 if not provided.
        speed_val = command_data.get("speed", 2)
        
        # Convert the command into a ROS Twist message based on the command type
        twist = Twist()
        cmd = command_data.get("command", "")
        
        if cmd == "go_forward":
            twist.linear.x = speed_val  # Use speed_val
            twist.angular.z = 0.0
            rospy.loginfo("Executing go_forward command with speed: %s", speed_val)
        elif cmd == "go_backwards":
            twist.linear.x = -speed_val  # Use -speed_val for backwards motion
            twist.angular.z = 0.0
            rospy.loginfo("Executing go_backwards command with speed: %s", speed_val)
        elif cmd == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            rospy.loginfo("Executing stop command")
            # Cancel any turning state
            turning = False
            initial_theta = None
            current_turn_threshold = None
        elif cmd == "turn_right_90":
            twist.linear.x = 0.0
            twist.angular.z = -0.5  # Negative angular for right turn
            current_turn_threshold = math.pi / 2  # 90 degrees
            turning = True
            initial_theta = None  # Will be set in the odom_turning callback
            rospy.loginfo("Executing turn_right_90 command")
        elif cmd == "turn_left_90":
            twist.linear.x = 0.0
            twist.angular.z = 0.5   # Positive angular for left turn
            current_turn_threshold = math.pi / 2  # 90 degrees
            turning = True
            initial_theta = None
            rospy.loginfo("Executing turn_left_90 command")
        elif cmd == "rotate":
            twist.linear.x = 0.0
            twist.angular.z = 0.5   # Adjust as needed (could be negative for right turn)
            current_turn_threshold = math.pi  # 180 degrees
            turning = True
            initial_theta = None
            rospy.loginfo("Executing rotate command")
        else:
            rospy.logwarn("Unknown command received: %s", cmd)
        
        # Convert the Twist message to a dictionary for MQTT transmission
        twist_dict = {
            "linear": {
                "x": twist.linear.x,
                "y": twist.linear.y,
                "z": twist.linear.z
            },
            "angular": {
                "x": twist.angular.x,
                "y": twist.angular.y,
                "z": twist.angular.z
            }
        }
        
        # Add timestamp and sequence number to the payload.
        current_time = time.time()  # Current processing time
        command_seq += 1           # Increment the global sequence counter
        twist_dict["timestamp"] = current_time
        twist_dict["seq"] = command_seq
        
        # Serialize the dictionary using CBOR and publish to the MQTT optimized command topic
        payload = json.dumps(twist_dict)
        mqttPublish(MQTT_OPTIMISED_COMMAND, payload, "command")
        rospy.loginfo("Published optimised command to MQTT: %s", twist_dict)
        
    except Exception as e:
        rospy.logerr("Error processing command: %s", e)


def on_publish(client, userdata, mid):
    global publish_timestamps, mqtt_publish_latencies
    if mid in publish_timestamps:
        publish_time = publish_timestamps.pop(mid)
        ack_time = rospy.Time.now().to_sec()
        latency = ack_time - publish_time
        mqtt_publish_latencies.append(latency)
        # rospy.loginfo("MQTT publish round-trip latency for mid %s: %.3f seconds", mid, latency)    

def calc_mqtt_latency():
    global mqqt_publish_latencies
    if mqtt_publish_latencies:
        avg_latency = sum(mqtt_publish_latencies) / len(mqtt_publish_latencies)
        jitter = statistics.stdev(mqtt_publish_latencies) if len(mqtt_publish_latencies) > 1 else 0.0
        # rospy.loginfo("MQTT Metrics: Avg Latency: %.3f ms, Count: %d",avg_latency*1000, len(mqtt_publish_latencies))
        return avg_latency, jitter, len(mqtt_publish_latencies)
    return 0, 0, 0

def calc_mqtt_bandwidth():
    global mqtt_total_bytes, mqtt_overall_bytes
    bandwidth_usage = mqtt_total_bytes
    totalBytes = mqtt_total_bytes + mqtt_overall_bytes
    return bandwidth_usage, totalBytes

def calc_mqtt_error_metrics():
    global mqtt_errors
    # Total publish attempts: successes + errors
    total_attempts = len(mqtt_publish_latencies) + mqtt_errors
    if total_attempts > 0:
        error_rate = (mqtt_errors / total_attempts) * 100.0
    else:
        error_rate = 0.0
    return error_rate, mqtt_errors, total_attempts

def calc_mqtt_throughput(interval):
    # Count of successful publishes is the number of latency measurements
    count = len(mqtt_publish_latencies)
    throughput = count / interval if interval > 0 else 0.0
    return throughput

def calc_mqtt_duplicate_count():
    global duplicate_count,message_hash_counts
    # Count of duplicate messages is the number of duplicates found
    total_messages = sum(message_hash_counts.values())
    duplicate_rate = (duplicate_count/total_messages) * 100.0 if total_messages > 0 else 0.0
    return duplicate_count, duplicate_rate

def calc_avg_payload_size():
    global mqtt_total_bytes, mqqt_publish_latencies
    count = len(mqtt_publish_latencies)
    if count > 0:
        avg_payload_size = mqtt_total_bytes / count
    else:
        avg_payload_size = 0
    return avg_payload_size

def get_cpu_usage():
    # Returns the current CPU usage percentage
    return psutil.cpu_percent(interval=None)

def get_memory_usage():
    # Returns the current memory usage percentage
    mem = psutil.virtual_memory()
    return mem.percent

def log_resource_utilization(event):
    cpu_usage = get_cpu_usage()           # e.g., 15.3 (%)
    memory_usage = get_memory_usage()     # e.g., 42.7 (%)#
    battery_level = getBatteryLevel()     # Assuming this returns a fraction (0 to 1)
    if(battery_level == None):
        battery_level = 100
    
    rospy.loginfo("Resource Utilization: CPU: %.1f%%, Memory: %.1f%%, Battery: %.1f%%",cpu_usage, memory_usage, battery_level * 100)


def log_mqtt_overall_metrics(event):
    """
    Aggregates MQTT metrics from the individual metric functions and logs them.
    Also resets the metrics for the next interval.
    So i have interval metrics and total metrics i need to differentiate them .. 
    """
    # Calculate time interval from the timer event
    interval = event.current_real.to_sec() - (
        event.last_real.to_sec() if event.last_real else event.current_real.to_sec() - 1.0
    )
    
    avg_latency, jitter, lengthOfLatency = calc_mqtt_latency()
    bandwidth_metrics, totalBytes = calc_mqtt_bandwidth()
    error_rate, errors, attempts  = calc_mqtt_error_metrics()
    throughput = calc_mqtt_throughput(interval)
    dupeCount, dupeRate = calc_mqtt_duplicate_count()
    avg_payload_size = calc_avg_payload_size()

    rospy.loginfo("----- MQTT Overall Metrics (Every 5 seconds) -----")
    rospy.loginfo("Total Messages Published: %d", lengthOfLatency)
    rospy.loginfo("Total Throughput: %.0f messages/s", throughput)
    rospy.loginfo("Latency AVG: %.3f ms",avg_latency * 1000)
    rospy.loginfo("Jitter Rate AVG: %.3f ms", jitter * 1000)
    rospy.loginfo("Total Bandwidth: %d bytes", totalBytes)
    rospy.loginfo("Bandwidth Rate: %d bytes/s", bandwidth_metrics/interval)
    rospy.loginfo("Average Payload Size: %d bytes", avg_payload_size)
    rospy.loginfo("Error Rate: %.3f%% (%d errors out of %d attempts)", error_rate, errors, attempts)
    rospy.loginfo("Duplicate Count: %d", dupeCount)
    rospy.loginfo("Duplicate Rate: %.3f%%", dupeRate)
    rospy.loginfo("--------------------------------")

    # Reset MQTT-specific metrics after logging
    global mqtt_total_bytes, mqtt_errors,mqtt_publish_latencies, duplicate_count
    mqtt_publish_latencies.clear()
    mqtt_total_bytes = 0
    mqtt_errors = 0
    duplicate_count = 0


# Setup MQTT client
mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
mqtt_client.on_message = on_mqtt_message
mqtt_client.on_publish = on_publish

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

def odom_turning_callback(odom_msg):
    global turning, initial_theta, current_turn_threshold, cmd_pub
    if not turning:
        return  # Not in turning mode; do nothing.
    
    # Extract current orientation (yaw) from the odometry message
    q = odom_msg.pose.pose.orientation
    # Use tf to convert quaternion to Euler angles; we only need yaw.
    (_, _, current_theta) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    
    if initial_theta is None:
        # Record the initial orientation when the turn starts.
        initial_theta = current_theta
        rospy.loginfo("Recorded initial orientation: %.3f radians", initial_theta)
    else:
        # Check if the robot has turned the required angle.
        if has_turned(initial_theta, current_theta, current_turn_threshold):
            rospy.loginfo("Turn complete: initial=%.3f, current=%.3f (threshold=%.3f)", 
                          initial_theta, current_theta, current_turn_threshold)
            # Publish a stop command to halt turning.
            stop_twist = Twist()
            stop_twist.linear.x = 0.0
            stop_twist.angular.z = 0.0
            cmd_pub.publish(stop_twist)
            # Reset turning state.
            turning = False
            initial_theta = None
            current_turn_threshold = None


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

# Global dictionary for sequence numbers per destination/topic.
sequence_numbers = {}

def mqttPublish(destination, msg, source=False, add_publish_time=True, add_sequence=True):
    global mqtt_total_bytes, mqtt_errors, sequence_numbers, publish_timestamps
    try:
        # Decode payload into a dictionary.
        try:
            data = json.loads(msg)
            if not isinstance(data, dict):
                data = {"payload": data}
        except Exception as e:
            rospy.logwarn("Failed to decode payload: %s. Wrapping raw payload.", e)
            data = {"payload": msg}
        
        # Inject publish_time if needed.
        if add_publish_time:
            data["publish_time"] = rospy.Time.now().to_sec()
        
        # Inject sequence number if needed.
        if add_sequence:
            seq = sequence_numbers.get(destination, 0)
            data["seq"] = seq
            sequence_numbers[destination] = seq + 1
        
        # Serialize the updated data back into msg.
        msg = json.dumps(data)
        
        # Publish the message.
        result, mid = mqtt_client.publish(destination, msg)
        publish_timestamps[mid] = rospy.Time.now().to_sec()
        
        # Update bandwidth counter.
        mqtt_total_bytes += len(msg)
        
        return result, mid
    except Exception as e:
        rospy.logerr("Error publishing to topic %s: %s", destination, e)
        mqtt_errors += 1


def helper(source,payload,latency,msg):
    
    sendBandwidth(source,payload)
    sendLatencyData(source,latency)
    try:
        mqttPublish(msg, payload, source)
    except Exception as e:
        rospy.logerr("Error publishing to topic %s for %s: %s", msg, source, e)
        error_counters[source] += 1

def write_metrics_to_file(metrics):
    with open("/home/abdullah/catkin_ws/src/synoptic-project-Abdash1234/mqtt_bridge/mqtt_bridge/scripts/metrics.txt", "a") as f:
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
    mqtt_client.publish(MQTT_THROUGHPUT, json.dumps(metrics))
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

def log_battery_metrics(event):
    global throughput_counters, bandwidth_counters, overall_bandwidth_usage

    # Determine the interval (in seconds)
    interval = event.current_real.to_sec() - (event.last_real.to_sec() if event.last_real else event.current_real.to_sec() - 1.0)


    # Sum up the total bytes transmitted for battery in this interval
    battery_bytes = bandwidth_counters["battery"]
    overall_bandwidth_usage = battery_bytes / interval  # bytes per second

    rospy.loginfo("Battery message published on 'robot/battery' payload size is %d bytes", battery_bytes)
    
    # Reset the battery counters for the next interval
    throughput_counters["battery"] = 0
    bandwidth_counters["battery"] = 0

def mqtt_bridge_node():
    rospy.init_node('mqtt_bridge', anonymous=True)
    # Subscribe to each ROS topic with its corresponding callback
    rospy.Subscriber("/battery", BatteryState, battery_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.Subscriber("/imu", Imu, imu_callback)
    rospy.Subscriber("/odom", Odometry, odom_turning_callback)


    # rospy.Timer(rospy.Duration(5.0), log_metrics)
    # rospy.Timer(rospy.Duration(5.0), log_mqtt_overall_metrics)
    # rospy.Timer(rospy.Duration(5.0), lambda event: log_resource_utilization(event))
    # rospy.Timer(rospy.Duration(6.0), log_battery_metrics)
    
    rospy.loginfo("MQTT Bridge node started. Bridging ROS topics to MQTT topics.")
    mqtt_client.loop_start()
    mqtt_client.subscribe(MQTT_RAW_COMMAND, 0)
    rospy.spin()
    mqtt_client.loop_stop()

def has_turned(initial, current, threshold):
    """
    Normalize the angular difference to [-pi, pi] and return True if
    the absolute difference is greater than or equal to threshold.
    """
    diff = current - initial
    diff = (diff + math.pi) % (2 * math.pi) - math.pi
    return abs(diff) >= threshold

if __name__ == '__main__':
    try:
        mqtt_bridge_node()
    except rospy.ROSInterruptException:
        pass
