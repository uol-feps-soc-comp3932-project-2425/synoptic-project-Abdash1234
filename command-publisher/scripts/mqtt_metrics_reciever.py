#!/usr/bin/env python3
import rospy
import paho.mqtt.client as mqtt
import cbor2
import json

# MQTT settings
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
# Topics to monitor.
TOPICS = ["robot/odom", "robot/scan", "robot/imu", "robot/battery"]

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

# Sequence tracking: expected sequence number per topic, and missed packet counters.
expected_seq = {topic: None for topic in TOPICS}
overall_missed_packets = {topic: 0 for topic in TOPICS}
interval_missed_packets = {topic: 0 for topic in TOPICS}

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

    try:
        # Try to decode using CBOR first.
        try:
            data = cbor2.loads(msg.payload)
            rospy.logdebug("Decoded message using CBOR on topic %s", msg.topic)
        except Exception as e:
            # rospy.logwarn("CBOR decode failed (%s), trying JSON...", e)
            data = json.loads(msg.payload.decode('utf-8'))
            rospy.logdebug("Decoded message using JSON on topic %s", msg.topic)
    except Exception as e:
        rospy.logerr("Failed to decode message: %s", e)
        return

    if msg.topic in TOPICS:
        # Update throughput metrics.
        payload_size = len(msg.payload)
        overall_throughput_counts[msg.topic] += 1
        overall_throughput_bytes[msg.topic] += payload_size
        interval_throughput_counts[msg.topic] += 1
        interval_throughput_bytes[msg.topic] += payload_size

        # Process sequence number to track lost packets.
        seq = data.get('seq', None)
        if seq is None:
            rospy.logwarn("No sequence number found in message on topic %s", msg.topic)
        else:
            if expected_seq[msg.topic] is None:
                # First message; set expected sequence to current + 1.
                expected_seq[msg.topic] = seq + 1
            else:
                if seq != expected_seq[msg.topic]:
                    # Calculate number of missed packets (assuming increasing sequence numbers).
                    missed = seq - expected_seq[msg.topic]
                    if missed < 0:
                        missed = 0  # In case of reordering.
                    overall_missed_packets[msg.topic] += missed
                    interval_missed_packets[msg.topic] += missed
                    rospy.logwarn("Topic %s: missed %d packets (expected %d, got %d)",
                                  msg.topic, missed, expected_seq[msg.topic], seq)
                    expected_seq[msg.topic] = seq + 1
                else:
                    expected_seq[msg.topic] = seq + 1

        # Process latency if publish_time is present.
        try:
            published_time = data.get('publish_time', None)
            if published_time is None:
                rospy.logwarn("No publish_time found in message on topic %s.", msg.topic)
                return

            current_time = rospy.get_time()
            latency = current_time - published_time

            # Update latency accumulators.
            overall_latency_sums[msg.topic] += latency
            overall_msg_counts[msg.topic] += 1
            interval_latency_sums[msg.topic] += latency
            interval_msg_counts[msg.topic] += 1

            # Compute jitter if a previous latency exists.
            if last_latency[msg.topic] is not None:
                jitter = abs(latency - last_latency[msg.topic])
                overall_jitter_sums[msg.topic] += jitter
                overall_jitter_counts[msg.topic] += 1
                interval_jitter_sums[msg.topic] += jitter
                interval_jitter_counts[msg.topic] += 1
            # Update last_latency with the current latency.
            last_latency[msg.topic] = latency

        except Exception as e:
            rospy.logerr("Error computing latency for %s: %s", msg.topic, e)

def timer_callback(event):
    """
    Every 5 seconds, log tidier metrics including latency, throughput, average payload size,
    jitter, and sequence error metrics for each topic and overall.
    """
    global interval_latency_sums, interval_msg_counts
    global interval_throughput_counts, interval_throughput_bytes
    global interval_jitter_sums, interval_jitter_counts
    global interval_missed_packets

    lines = []
    lines.append("----- 5-second Interval Metrics -----")
    total_msgs = 0
    total_latency = 0.0
    total_throughput_msgs = 0
    total_throughput_bytes = 0
    total_jitter_sum = 0.0
    total_jitter_count = 0
    total_missed = 0

    for topic in TOPICS:
        msg_count = interval_msg_counts[topic]
        throughput_msgs = interval_throughput_counts[topic]
        throughput_bytes = interval_throughput_bytes[topic]
        jitter_sum = interval_jitter_sums[topic]
        jitter_count = interval_jitter_counts[topic]
        missed = interval_missed_packets[topic]

        # Per-topic metrics.
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

        error_rate = (missed / (throughput_msgs + missed) * 100) if (throughput_msgs + missed) > 0 else 0

        lines.append(f"Topic: {topic}")
        lines.append(f"  Messages      : {msg_count}")
        lines.append(f"  Avg Latency   : {avg_latency*1000:.6f} ms")
        lines.append(f"  Throughput    : {throughput_msgs} msgs ({msgs_per_sec:.2f} msgs/sec)")
        lines.append(f"                  {throughput_bytes} bytes ({bytes_per_sec:.2f} bytes/sec)")
        lines.append(f"  Avg Payload   : {avg_payload_size:.2f} bytes")
        lines.append(f"  Avg Jitter    : {avg_jitter*1000:.6f} ms (over {jitter_count} differences)")
        lines.append(f"  Missed Packets: {missed} (Error Rate: {error_rate:.2f}%)")
        lines.append("-" * 40)

        total_msgs += msg_count
        total_latency += interval_latency_sums[topic]
        total_throughput_msgs += throughput_msgs
        total_throughput_bytes += throughput_bytes
        total_jitter_sum += jitter_sum
        total_jitter_count += jitter_count
        total_missed += missed

    if total_msgs > 0:
        overall_avg_latency = total_latency / total_msgs
    else:
        overall_avg_latency = 0.0

    overall_msgs_per_sec = total_throughput_msgs / 5.0
    overall_bytes_per_sec = total_throughput_bytes / 5.0
    overall_avg_payload_size = (total_throughput_bytes / total_throughput_msgs) if total_throughput_msgs > 0 else 0
    overall_avg_jitter = (total_jitter_sum / total_jitter_count) if total_jitter_count > 0 else 0.0
    overall_expected = total_throughput_msgs + total_missed
    overall_error_rate = (total_missed / overall_expected * 100) if overall_expected > 0 else 0

    lines.append("OVERALL SUMMARY")
    lines.append(f"  Total Messages    : {total_throughput_msgs}")
    lines.append(f"  Overall Latency   : {overall_avg_latency*1000:.6f} ms")
    lines.append(f"  Throughput        : {total_throughput_msgs} msgs ({overall_msgs_per_sec:.2f} msgs/sec), "
                 f"{total_throughput_bytes} bytes ({overall_bytes_per_sec:.2f} bytes/sec)")
    lines.append(f"  Avg Payload Size  : {overall_avg_payload_size:.2f} bytes")
    lines.append(f"  Overall Jitter    : {overall_avg_jitter*1000:.6f} ms (over {total_jitter_count} differences)")
    lines.append(f"  Missed Packets    : {total_missed} (Error Rate: {overall_error_rate:.2f}%)")
    lines.append("----------------------------------------")

    # Log the entire block as one info message.
    rospy.loginfo("\n".join(lines))

    # Reset interval accumulators for next interval.
    interval_latency_sums = {topic: 0.0 for topic in TOPICS}
    interval_msg_counts = {topic: 0 for topic in TOPICS}
    interval_throughput_counts = {topic: 0 for topic in TOPICS}
    interval_throughput_bytes = {topic: 0 for topic in TOPICS}
    interval_jitter_sums = {topic: 0.0 for topic in TOPICS}
    interval_jitter_counts = {topic: 0 for topic in TOPICS}
    interval_missed_packets = {topic: 0 for topic in TOPICS}


def mqtt_subscriber_node():
    rospy.init_node('mqtt_subscriber_node', anonymous=True)

    # Timer fires every 5 seconds.
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
