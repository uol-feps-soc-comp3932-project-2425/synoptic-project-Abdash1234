#!/usr/bin/env python3
import json, rospy
import paho.mqtt.client as mqtt
from sensor_msgs.msg import BatteryState, Imu, LaserScan
from nav_msgs.msg import Odometry
import statistics, math, tf
import cbor2, hashlib, psutil, threading, time
from geometry_msgs.msg import Twist


class EventManager:
    def __init__(self):
        self.listeners = {}

    def register(self, event_name, callback):
        if event_name not in self.listeners:
            self.listeners[event_name] = []
        self.listeners[event_name].append(callback)

    def trigger(self, event_name, *args, **kwargs):
        if event_name in self.listeners:
            for callback in self.listeners[event_name]:
                callback(*args, **kwargs)

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

latest_battery_percentage = None
overall_mean_latency = None
overall_processing_stdev = None
overall_error_rate = None
overall_bandwidth_usage = None


turning = False
initial_theta = None
current_turn_threshold = None  # In radians
event_manager = EventManager()



# Global throughput counters (messages counted per topic)
throughput_counters = {
    "battery": 0,
    "odom": 0,
    "scan": 0,
    "imu": 0,
}

# Global Aggregation Buffers (for storing messages before sending)
battery_buffer = []

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
qos_decision_log = []  # Global list to store QoS decision logs
current_qos = 0
qos_lock = threading.Lock()
MQTT_RAW_COMMAND = "raw/command"
cmd_pub = 0


def on_high_latency(topic, latency):
    rospy.logwarn("High latency event: topic '%s' latency = %.3f sec", topic, latency)
    global current_qos
    current_qos = 2
    rospy.loginfo("QoS updated to: %d", current_qos)

event_manager.register("high_latency", on_high_latency)

# MQTT callback for incoming messages (if needed)
def on_mqtt_message(client, userdata, msg):
    if msg.topic == MQTT_RAW_COMMAND:
        readMessage(msg)
    global duplicate_count
    # Compute hash for the incoming message payload
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
    global mqtt_client, turning, initial_theta, current_turn_threshold, cmd_pub
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
            # cmd_pub.publish(twist)
        elif cmd == "go_backwards":
            twist.linear.x = -speed_val  # Use -speed_val for backwards motion
            twist.angular.z = 0.0
            rospy.loginfo("Executing go_backwards command with speed: %s", speed_val)
            # cmd_pub.publish(twist)
        elif cmd == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            rospy.loginfo("Executing stop command")
            # cmd_pub.publish(twist)
            # Also cancel any turning state
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
            # cmd_pub.publish(twist)
        elif cmd == "turn_left_90":
            twist.linear.x = 0.0
            twist.angular.z = 0.5   # Positive angular for left turn
            current_turn_threshold = math.pi / 2  # 90 degrees
            turning = True
            initial_theta = None
            rospy.loginfo("Executing turn_left_90 command")
            # cmd_pub.publish(twist)
        elif cmd == "rotate":
            twist.linear.x = 0.0
            twist.angular.z = 0.5   # Adjust as needed (could be negative for right turn)
            current_turn_threshold = math.pi  # 180 degrees
            turning = True
            initial_theta = None
            rospy.loginfo("Executing rotate command")
            # cmd_pub.publish(twist)
        else:
            rospy.logwarn("Unknown command received: %s", cmd)
        
        # Convert the Twist message to a dictionary for MQTT transmission (if needed)
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
        
        # Serialize the dictionary using CBOR and publish to the MQTT optimized command topic
        payload = cbor2.dumps(twist_dict)
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
    local_latencies = mqtt_publish_latencies.copy()
    if mqtt_publish_latencies:
        avg_latency = sum(local_latencies) / len(local_latencies)
        jitter = statistics.stdev(local_latencies) if len(local_latencies) > 1 else 0.0
        return avg_latency, jitter, len(local_latencies)
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

def calc_aggregation_efficiency(buffer):
    if buffer >0:
        return ((buffer - 1)/ float(buffer)) * 100
    return 0

def writeQoSLog():
    with open("/home/abdullah/catkin_ws/src/synoptic-project-Abdash1234/mqtt_optimised/scripts/qos_decision_log.txt", "a") as f:
        # Convert the metrics dictionary to a formatted JSON string
        qos_decision_str = json.dumps(qos_decision_log, indent=2)
        # Write the metrics with a newline separator
        f.write(qos_decision_str + "\n")

def log_mqtt_overall_metrics(event):

    global battery_buffer

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
    battery_agg_efficiency = calc_aggregation_efficiency(len(battery_buffer))   

    rospy.loginfo("----- MQTT Overall Metrics (Every 5 seconds) -----")
    rospy.loginfo("Total Messages Published: %d", lengthOfLatency)
    rospy.loginfo("Total Throughput: %.0f messages/s", throughput)
    rospy.loginfo("Latency AVG: %.3f ms",avg_latency * 1000)
    rospy.loginfo("Jitter Rate AVG: %.3f ms", jitter * 1000)
    rospy.loginfo("Total Bandwidth: %d bytes", totalBytes)
    rospy.loginfo("Bandwidth Rate: %d bytes/s", bandwidth_metrics/interval)
    rospy.loginfo("Average Payload Size: %d bytes", avg_payload_size)
    rospy.loginfo("Error Rate: %.0f%% (%d errors out of %d attempts)", error_rate, errors, attempts)
    rospy.loginfo("Duplicate Count: %d", dupeCount)
    rospy.loginfo("Duplicate Rate: %.0f%%", dupeRate)
    rospy.loginfo("Battery Aggregation Efficiency: %.0f%%, (aggregated %d messages into 1)", battery_agg_efficiency, len(battery_buffer))
    rospy.loginfo("--------------------------------")

    # Reset MQTT-specific metrics after logging
    global mqtt_total_bytes, mqtt_errors,mqtt_publish_latencies, duplicate_count
    mqtt_publish_latencies.clear()
    battery_buffer.clear()
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

    latency_threshold = 0.1
    if latency > latency_threshold:
        event_manager.trigger("high_latency", "battery", latency)


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
    battery_buffer.append(payload)
    # helper("battery",payload,latency,MQTT_BATTERY)
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

def getBatteryLevel():
    global latest_battery_percentage
    if latest_battery_percentage != None:
        return latest_battery_percentage
    else:
        rospy.logwarn("Battery level not available")
        return -1
    
def getCurrentAverageLatency():
    return overall_mean_latency

def defineEvents():
    # Define events based on the metrics
    # For example, if the latency is above a certain threshold, trigger an event
    # This is a placeholder function. You can implement your own event definitions.

    #Event 1: -> battery levels drop by 5-10% between intervals
    #Event 2: -> battery levels fall below a critical threshold - sends all the messages instantly
    return 0

def normaliseLatency(latency, minLatency = 0.001, maxLatency = 0.1):
    # Normalise the latency value to a range of 0 to 1
    if latency < minLatency:
        return 0.0
    elif latency > maxLatency:
        return 1.0
    else:
        return (latency - minLatency) / (maxLatency - minLatency)

def normaliseBandwidth(bandwidth, low_bw = 1000, high_bw = 100000):
    if bandwidth <= low_bw:
        return 0.0
    elif bandwidth >= high_bw:
        return 1.0
    else:
        return (bandwidth - low_bw) / (high_bw - low_bw)
    
def normaliseJitter(jitter, minJitter = 0, maxJitter = 0.005):
    if jitter < minJitter:
        return 0.0
    elif jitter > maxJitter:
        return 1.0
    else:
        return (jitter - minJitter) / (maxJitter - minJitter)

def calcScore(batteryLevel, latency, bandwidth, jitter, errorRate):
    # Normalise the values
    calcBatteryValue = 1.0 - (batteryLevel)
    calcLatency = normaliseLatency(latency)
    calcErrorRate = errorRate / 100.0
    calcBandwidth = normaliseBandwidth(bandwidth)
    calcjitter = normaliseJitter(jitter)

    # Set Weights
    w_battery = 0.3
    w_latency = 0.35
    w_error   = 0.2
    w_bw      = 0.1
    w_jitter  = 0.05

    # Compute the weighted sum score
    score = (w_battery * calcBatteryValue +
             w_latency * calcLatency +
             w_error   * calcErrorRate +
             w_bw      * calcBandwidth +
             w_jitter  * calcjitter)

    return score

def setQoS():
    chosen_qos = 0

    battery_percentage = getBatteryLevel()
    mqtt_avg_latency, mqtt_jitter, mqtt_length = calc_mqtt_latency()
    mqtt_bandwidth_usage, _ = calc_mqtt_bandwidth()
    mqtt_error_rate, mqtt_errors, mqtt_attempts = calc_mqtt_error_metrics()

    # Check if all metrics are zero (i.e., no data has been collected)
    if mqtt_avg_latency == 0 and mqtt_bandwidth_usage == 0 and mqtt_error_rate == 0 and mqtt_jitter == 0 and mqtt_errors == 0:
        qos_decision_log.append({"test": "no metrics available"})
        return 0

    # Compute score using MQTT metrics
    score = calcScore(battery_percentage, mqtt_avg_latency, mqtt_bandwidth_usage, mqtt_jitter, mqtt_error_rate)
    if score < 0.3:
        chosen_qos = 0
    elif score < 0.6:
        chosen_qos = 1
    else:
        chosen_qos = 2

    qos_decision_log.append({
        "timestamp": float(rospy.Time.now().to_sec()),
        "battery": float(battery_percentage),
        "mqtt_avg_latency": float(mqtt_avg_latency),
        "mqtt_bandwidth_usage": float(mqtt_bandwidth_usage),
        "mqtt_jitter": float(mqtt_jitter),
        "mqtt_error_rate": float(mqtt_error_rate),
        "score": float(score),
        "chosen_qos": int(chosen_qos)
    })

    return chosen_qos

def qos_updater():
    global current_qos
    while not rospy.is_shutdown():
        # Compute the new QoS value using your existing function.
        # (This assumes setQoS() returns an integer QoS value.)
        new_qos = setQoS()
        with qos_lock:
            current_qos = new_qos
        rospy.loginfo("Updated QoS to: %d", new_qos)
        time.sleep(2)  # Update every 2 seconds (adjust as needed)

def mqttPublish(destination, msg, source=False):
    # qosValue = setQoS()
    global current_qos
    with qos_lock:
        qosValue = current_qos if current_qos is not None else 0
    try:
        result, mid = mqtt_client.publish(destination, msg, qos=qosValue)
        publish_timestamps[mid] = rospy.Time.now().to_sec()
        # Also update bandwidth counter (see next section)
        global mqtt_total_bytes
        mqtt_total_bytes += len(msg)
    except Exception as e:
        rospy.logerr("Error publishing to topic %s: %s", destination, e)
        # Record error for MQTT-specific metrics
        global mqtt_errors
        mqtt_errors += 1

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

def publish_aggregated_buffer(buffer, topic):
    total_msgs = len(buffer)
    if total_msgs == 0:
        rospy.loginfo("No messages to aggregate for topic '%s'", topic)
        return

    # Convert the list of messages (which are already CBOR-encoded bytes) into a single aggregated payload.
    aggregated_payload = cbor2.dumps(buffer)
    aggregated_payload_size = len(aggregated_payload)

    # Publish the aggregated payload on the given MQTT topic.
    mqttPublish(topic, aggregated_payload)

    # Log the details.
    rospy.loginfo("Aggregated message published on '%s' payload size is %d bytes", topic, aggregated_payload_size)

    # Clear the buffer for the next interval.
    buffer.clear()

def write_metrics_to_file(metrics):
    with open("/home/abdullah/catkin_ws/src/synoptic-project-Abdash1234/mqtt_optimised/scripts/metrics.txt", "a") as f:
        # Convert the metrics dictionary to a formatted JSON string
        metrics_str = json.dumps(metrics, indent=2)
        # Write the metrics with a newline separator
        f.write(metrics_str + "\n")


    metrics = {
        "throughput": throughput_data,
        "latency": latency_stats,
        "processing": processing_stats,
        "errors": error_percentages,
    }

    rospy.loginfo("Average Latency: %.3f ms", overall_mean_latency*1000)
    rospy.loginfo("Current Battery Levels : %.3f", getBatteryLevel()*100)
    rospy.loginfo("Overall Processing Jitter (stdev): %fms", overall_processing_stdev)
    rospy.loginfo("Overall Bandwidth Usage: %.3f bytes/s", (overall_bandwidth_usage/5))
    rospy.loginfo("Overall Error Rate: %.3f%%", overall_error_rate)
    # rospy.loginfo("Metrics: %s", metrics)
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

def has_turned(initial, current, threshold):
    """
    Normalize the angular difference to [-pi, pi] and return True if
    the absolute difference is greater than or equal to threshold.
    """
    diff = current - initial
    diff = (diff + math.pi) % (2 * math.pi) - math.pi
    return abs(diff) >= threshold


def mqtt_bridge_node():
    global cmd_pub
    rospy.init_node('mqtt_bridge', anonymous=True)

    qos_thread = threading.Thread(target=qos_updater, daemon=True)
    qos_thread.start()
    # Subscribe to each ROS topic with its corresponding callback
    rospy.Subscriber("/battery", BatteryState, battery_callback)
    rospy.Subscriber("/odom", Odometry, odom_callback)
    rospy.Subscriber("/scan", LaserScan, scan_callback)
    rospy.Subscriber("/imu", Imu, imu_callback)
    rospy.Subscriber("/odom", Odometry, odom_turning_callback)


    # cmd_pub = rospy.Publisher('/mobile_base/cmd_vel', Twist, queue_size=10)

    # rospy.Timer(rospy.Duration(4), lambda event: publish_aggregated_buffer(battery_buffer, MQTT_BATTERY))


    rospy.Timer(rospy.Duration(5.0), log_mqtt_overall_metrics)
    rospy.Timer(rospy.Duration(5.0), lambda event: log_resource_utilization(event))
    # rospy.Timer(rospy.Duration(5.0), lambda event: writeQoSLog())
    
    
    rospy.loginfo("MQTT Bridge node started. Bridging ROS topics to MQTT topics.")
    mqtt_client.loop_start()
    mqtt_client.subscribe(MQTT_RAW_COMMAND, 0)
    rospy.spin()
    mqtt_client.loop_stop()



if __name__ == '__main__':
    try:
        mqtt_bridge_node()
    except rospy.ROSInterruptException:
        pass
