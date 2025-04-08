#!/usr/bin/env python3
import rospy
import paho.mqtt.client as mqtt
import cbor2
import json
import csv, os

# MQTT settings
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
CSV_FILE = "/home/abdullah/catkin_ws/src/synoptic-project-Abdash1234/csv_files/mqtt_metrics.csv"

# Topics to monitor.
# "robot/battery_status" is used exclusively for battery percentage;
# "mqtt/summary" is used to receive the publisher summary.
# The remaining topics collect sensor metrics.
TOPICS = ["robot/odom", "robot/scan", "robot/imu", "robot/battery",
          "robot/battery_status", "mqtt/summary"]

# Global list to store battery percentage values from battery_status topic.
battery_records = []

# Global variable to store the latest publisher summary.
publisher_summary = None

# Latency accumulators
overall_latency_sums = {topic: 0.0 for topic in TOPICS}
overall_msg_counts = {topic: 0 for topic in TOPICS}
interval_latency_sums = {topic: 0.0 for topic in TOPICS}
interval_msg_counts = {topic: 0 for topic in TOPICS}

# Throughput accumulators: message count and total payload bytes per topic
overall_throughput_counts = {topic: 0 for topic in TOPICS}
overall_throughput_bytes = {topic: 0 for topic in TOPICS}
interval_throughput_counts = {topic: 0 for topic in TOPICS}
interval_throughput_bytes = {topic: 0 for topic in TOPICS}

# Jitter accumulators: sum of jitter differences and count of differences per topic
overall_jitter_sums = {topic: 0.0 for topic in TOPICS}
overall_jitter_counts = {topic: 0 for topic in TOPICS}
interval_jitter_sums = {topic: 0.0 for topic in TOPICS}
interval_jitter_counts = {topic: 0 for topic in TOPICS}

# To compute jitter, store the last latency value per topic
last_latency = {topic: None for topic in TOPICS}

# Sequence tracking: (not used for loss calculation now)
expected_seq = {topic: None for topic in TOPICS}
overall_missed_packets = {topic: 0 for topic in TOPICS}
interval_missed_packets = {topic: 0 for topic in TOPICS}

# Global variable for battery percentage from "robot/battery" topic is not used for percentage now
latest_battery_percentage = None

def log_metrics_to_csv(metrics, csv_file=CSV_FILE):
    header = [
        "timestamp", 
        "battery_pct", 
        "overall_latency_ms", 
        "overall_jitter_ms", 
        "throughput_msgs_sec", 
        "avg_payload_bytes", 
        "lost_packets", 
        "error_rate_pct", 
        "processing_time_ms", 
        "bandwidth_bytes_sec"
    ]
    file_exists = os.path.exists(csv_file) and os.path.getsize(csv_file) > 0
    current_time = rospy.get_time()
    row = [
        current_time,
        metrics.get("battery_pct", "N/A"),
        metrics.get("overall_latency_ms", 0),
        metrics.get("overall_jitter_ms", 0),
        metrics.get("throughput_msgs_sec", 0),
        metrics.get("avg_payload_bytes", 0),
        metrics.get("lost_packets", 0),
        metrics.get("error_rate_pct", 0),
        metrics.get("processing_time_ms", 0),
        metrics.get("bandwidth_bytes_sec", 0)
    ]
    
    try:
        with open(csv_file, mode="a", newline="") as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(header)
            writer.writerow(row)
        rospy.loginfo("Logged metrics to CSV.")
    except Exception as e:
        rospy.logerr("Error writing to CSV: %s", e)

def on_connect(client, userdata, flags, rc):
    rospy.loginfo("Connected to MQTT Broker with result code %s", rc)
    for topic in TOPICS:
        client.subscribe(topic)
        rospy.loginfo("Subscribed to topic: %s", topic)

def on_message(client, userdata, msg):
    global overall_latency_sums, overall_msg_counts, interval_latency_sums, interval_msg_counts
    global overall_throughput_counts, overall_throughput_bytes, interval_throughput_counts, interval_throughput_bytes
    global overall_jitter_sums, overall_jitter_counts, interval_jitter_sums, interval_jitter_counts
    global last_latency, expected_seq, overall_missed_packets, interval_missed_packets
    global latest_battery_percentage, battery_records, publisher_summary

    try:
        try:
            data = cbor2.loads(msg.payload)
            rospy.logdebug("Decoded message using CBOR on topic %s", msg.topic)
        except Exception as e:
            data = json.loads(msg.payload.decode('utf-8'))
            rospy.logdebug("Decoded message using JSON on topic %s", msg.topic)
    except Exception as e:
        rospy.logerr("Failed to decode message: %s", e)
        return

    # Process summary messages from publisher (mqtt/summary)
    if msg.topic == "mqtt/summary":
        # Store the publisher summary in a global variable.
        publisher_summary = data.get("topics", {})
        rospy.loginfo("Received publisher summary: %s", publisher_summary)
        return

    # Process battery status messages separately for battery percentage averaging.
    if msg.topic == "robot/battery_status":
        battery_pct = data.get("percentage", None)
        if battery_pct is not None:
            battery_records.append(battery_pct)
        return

    # Process all other topics.
    if msg.topic in TOPICS:
        payload_size = len(msg.payload)
        overall_throughput_counts[msg.topic] += 1
        overall_throughput_bytes[msg.topic] += payload_size
        interval_throughput_counts[msg.topic] += 1
        interval_throughput_bytes[msg.topic] += payload_size

        # For "robot/battery", you may process additional metrics if needed (excluding percentage).
        if msg.topic == "robot/battery":
            pass

        # Here we assume sequence-based loss is not our primary method.
        # (You can remove sequence-based loss calculation if you use publisher summary instead.)
        seq = data.get('seq', None)
        if seq is not None:
            if expected_seq.get(msg.topic) is None:
                expected_seq[msg.topic] = seq + 1
            else:
                gap = seq - expected_seq[msg.topic]
                if gap > 0:
                    overall_missed_packets[msg.topic] += gap
                    interval_missed_packets[msg.topic] += gap
                    rospy.logwarn("Topic %s: missed %d packets (expected %d, got %d)",
                                  msg.topic, gap, expected_seq[msg.topic], seq)
                expected_seq[msg.topic] = seq + 1

        # Process latency.
        try:
            published_time = data.get('publish_time', None)
            if published_time is None:
                rospy.logwarn("No publish_time found in message on topic %s.", msg.topic)
                return
            current_time = rospy.get_time()
            latency = current_time - published_time

            overall_latency_sums[msg.topic] += latency
            overall_msg_counts[msg.topic] += 1
            interval_latency_sums[msg.topic] += latency
            interval_msg_counts[msg.topic] += 1

            if last_latency.get(msg.topic) is not None:
                jitter = abs(latency - last_latency[msg.topic])
                overall_jitter_sums[msg.topic] += jitter
                overall_jitter_counts[msg.topic] += 1
                interval_jitter_sums[msg.topic] += jitter
                interval_jitter_counts[msg.topic] += 1
            last_latency[msg.topic] = latency
        except Exception as e:
            rospy.logerr("Error computing latency for %s: %s", msg.topic, e)

def timer_callback(event):
    global interval_latency_sums, interval_msg_counts
    global interval_throughput_counts, interval_throughput_bytes
    global interval_jitter_sums, interval_jitter_counts
    global interval_missed_packets, battery_records, publisher_summary

    lines = []
    lines.append("----- 5-second Interval Metrics -----")
    total_msgs = 0
    total_latency = 0.0
    total_throughput_msgs = 0
    total_throughput_bytes = 0
    total_jitter_sum = 0.0
    total_jitter_count = 0
    total_lost = 0
    total_published = 0  # Accumulate published count from publisher summary

    # For each topic (except battery_status and summary) calculate metrics and error per topic.
    for topic in TOPICS:
        if topic in ["robot/battery_status", "mqtt/summary"]:
            continue

        msg_count = interval_msg_counts[topic]
        throughput_msgs = interval_throughput_counts[topic]
        throughput_bytes = interval_throughput_bytes[topic]
        jitter_sum = interval_jitter_sums[topic]
        jitter_count = interval_jitter_counts[topic]

        # Get the publisher's count for this topic from the summary.
        published_count = None
        if publisher_summary is not None and topic in publisher_summary:
            published_count = publisher_summary[topic].get("published_count", None)

        # Calculate loss for this topic using publisher summary if available.
        if published_count is not None:
            loss = published_count - throughput_msgs if published_count > throughput_msgs else 0
            total_published += published_count
        else:
            # If no publisher summary is available, fall back to sequence method (or assume loss = 0)
            loss = 0

        total_lost += loss

        if msg_count > 0:
            avg_latency = interval_latency_sums[topic] / msg_count
        else:
            avg_latency = 0.0

        if throughput_msgs > 0:
            msgs_per_sec = throughput_msgs / 5.0
            bytes_per_sec = throughput_bytes / 5.0
            avg_payload_size = throughput_bytes / throughput_msgs
        else:
            msgs_per_sec = 0
            bytes_per_sec = 0
            avg_payload_size = 0

        if jitter_count > 0:
            avg_jitter = jitter_sum / jitter_count
        else:
            avg_jitter = 0.0

        # Calculate per-topic error rate.
        error_rate = (loss / published_count * 100) if (published_count and published_count > 0) else 0

        lines.append(f"Topic: {topic}")
        lines.append(f"  Messages Received: {interval_msg_counts[topic]}")
        lines.append(f"  Published Count  : {published_count if published_count is not None else 'N/A'}")
        lines.append(f"  Avg Latency      : {avg_latency*1000:.6f} ms")
        lines.append(f"  Throughput       : {throughput_msgs} msgs ({msgs_per_sec:.2f} msgs/sec)")
        lines.append(f"                   : {throughput_bytes} bytes ({bytes_per_sec:.2f} bytes/sec)")
        lines.append(f"  Avg Payload      : {avg_payload_size:.2f} bytes")
        lines.append(f"  Avg Jitter       : {avg_jitter*1000:.6f} ms (over {jitter_count} differences)")
        lines.append(f"  Lost Packets     : {loss} (Error Rate: {error_rate:.2f}%)")
        lines.append("-" * 40)

        total_msgs += interval_msg_counts[topic]
        total_latency += interval_latency_sums[topic]
        total_throughput_msgs += throughput_msgs
        total_throughput_bytes += throughput_bytes
        total_jitter_sum += jitter_sum
        total_jitter_count += jitter_count

    if total_msgs > 0:
        overall_avg_latency = total_latency / total_msgs
    else:
        overall_avg_latency = 0.0

    overall_msgs_per_sec = total_throughput_msgs / 5.0
    overall_bytes_per_sec = total_throughput_bytes / 5.0
    overall_avg_payload_size = (total_throughput_bytes / total_throughput_msgs) if total_throughput_msgs > 0 else 0
    overall_avg_jitter = (total_jitter_sum / total_jitter_count) if total_jitter_count > 0 else 0.0

    # Overall error rate computed across all topics.
    if total_published > 0:
        overall_error_rate = (total_lost / total_published) * 100
    else:
        overall_error_rate = 0

    lines.append("OVERALL SUMMARY")
    lines.append(f"  Total Received Messages: {total_throughput_msgs}")
    lines.append(f"  Total Published Count  : {total_published if total_published > 0 else 'N/A'}")
    lines.append(f"  Overall Lost Packets   : {total_lost}")
    lines.append(f"  Overall Error Rate     : {overall_error_rate:.2f}%")
    lines.append(f"  Overall Latency        : {overall_avg_latency*1000:.6f} ms")
    lines.append(f"  Throughput             : {total_throughput_msgs} msgs ({overall_msgs_per_sec:.2f} msgs/sec), "
                 f"{total_throughput_bytes} bytes ({overall_bytes_per_sec:.2f} bytes/sec)")
    lines.append(f"  Avg Payload Size       : {overall_avg_payload_size:.2f} bytes")
    lines.append(f"  Overall Avg Jitter     : {overall_avg_jitter*1000:.6f} ms")
    lines.append("----------------------------------------")
    
    # Compute the average battery percentage from battery_status messages.
    if battery_records:
        avg_battery_pct = sum(battery_records) / len(battery_records)
    else:
        avg_battery_pct = None

    if avg_battery_pct is not None:
        battery_str = f"Battery Percentage: {avg_battery_pct*100:.2f}%"
    else:
        battery_str = "Battery Percentage: N/A"
    lines.append(battery_str)
    lines.append("----------------------------------------")
    rospy.loginfo("\n".join(lines))

    overall_metrics = {
        "battery_pct": (avg_battery_pct * 100) if avg_battery_pct is not None else "N/A",
        "overall_latency_ms": overall_avg_latency * 1000,
        "overall_jitter_ms": overall_avg_jitter * 1000,
        "throughput_msgs_sec": overall_msgs_per_sec,
        "avg_payload_bytes": overall_avg_payload_size,
        "lost_packets": overall_error_rate,  # Overall error rate as a percent.
        "error_rate_pct": overall_error_rate,
        "processing_time_ms": 0,  # Set to 0 or update if tracking overall processing time.
        "bandwidth_bytes_sec": overall_bytes_per_sec
    }
    
    log_metrics_to_csv(overall_metrics)
    print(overall_metrics)

    # Reset interval accumulators for next interval.
    interval_latency_sums = {topic: 0.0 for topic in TOPICS}
    interval_msg_counts = {topic: 0 for topic in TOPICS}
    interval_throughput_counts = {topic: 0 for topic in TOPICS}
    interval_throughput_bytes = {topic: 0 for topic in TOPICS}
    interval_jitter_sums = {topic: 0.0 for topic in TOPICS}
    interval_jitter_counts = {topic: 0 for topic in TOPICS}
    interval_missed_packets = {topic: 0 for topic in TOPICS}
    
    # Clear battery_records for next interval.
    battery_records = []

    # Optionally reset publisher_summary if desired.
    # publisher_summary = None


def mqtt_subscriber_node():
    rospy.init_node('mqtt_subscriber_node', anonymous=True)
    rospy.Timer(rospy.Duration(5.0), timer_callback)
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_start()
    rospy.loginfo("MQTT Subscriber node started. Listening to topics: %s", TOPICS)
    rospy.spin()
    client.loop_stop()

if __name__ == '__main__':
    try:
        mqtt_subscriber_node()
    except rospy.ROSInterruptException:
        pass
