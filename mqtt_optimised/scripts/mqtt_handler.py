#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import json, cbor2, rospy
import logging
import hashlib
from config import MQTT_BROKER, MQTT_PORT, MQTT_RAW_COMMAND

class MQTTHandler:
    def __init__(self, broker=MQTT_BROKER, port=MQTT_PORT, topics=None, movement_controller=None, metrics=None, qos=None):
        # Initialize MQTT broker connection details
        self.broker = broker
        self.port = port
        self.topics = topics  # Dict containing "subscribe" and "publish" keys
        self.movement_controller = movement_controller
        self.metrics = metrics  # Optional: for tracking performance
        self.qos_manager = qos  # Optional: adaptive QoS control
        self.client = mqtt.Client()
        self._setup_callbacks()  # Set connect, message, and publish callbacks

        # Track sequence numbers and message count per topic
        if topics and "publish" in topics:
            self.sequence_numbers = {topic: 0 for topic in topics["publish"]}
            self.published_message_counts = {topic: 0 for topic in topics["publish"]}
        else:
            self.sequence_numbers = {}
            self.published_message_counts = {}

        self.published_message_count = 0  # Global publish counter

    def _setup_callbacks(self):
        # Assign MQTT client callbacks
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_publish = self.on_publish

    def on_connect(self, client, userdata, flags, rc):
        # Subscribe to topics on successful broker connection
        logging.info("Connected with result code %s", rc)
        for topic in self.topics.get("subscribe", []):
            client.subscribe(topic)

    def on_message(self, client, userdata, msg):
        # Handles incoming MQTT messages
        if self.metrics:
            # Track duplicate messages using message hash
            payload_hash = hashlib.sha256(msg.payload).hexdigest()
            if payload_hash in self.metrics.message_hash_counts:
                self.metrics.message_hash_counts[payload_hash] += 1
                self.metrics.duplicate_count += 1  # Treat all beyond first as duplicates
            else:
                self.metrics.message_hash_counts[payload_hash] = 1

        # Handle movement commands
        if msg.topic == MQTT_RAW_COMMAND:
            try:
                print("1")
                command_data = json.loads(msg.payload.decode('utf-8'))
                if self.movement_controller:
                    print("2")
                    self.movement_controller.process_command(command_data)
                else:
                    logging.error("Movement controller is not set!")
                    print("3 - error")
            except Exception as e:
                logging.error("Error processing movement command: %s", e)

    def connect(self):
        # Connect to MQTT broker and start network loop
        self.client.connect(self.broker, self.port, 60)
        self.client.loop_start()

    def publish(self, topic, payload, qos=None, add_publish_time=True, add_sequence=True):
        # Safely decode payload to dict or wrap if decoding fails
        try:
            data = cbor2.loads(payload)
            if not isinstance(data, dict):
                data = {"payload": data}
        except Exception as e:
            rospy.logwarn("Failed to decode payload: %s. Wrapping raw payload.", e)
            data = {"payload": payload}

        # Add timestamp if requested
        if add_publish_time:
            data["publish_time"] = rospy.Time.now().to_sec()

        # Add and increment sequence number if enabled
        if add_sequence:
            seq = self.sequence_numbers.get(topic, 0)
            data["seq"] = seq
            self.sequence_numbers[topic] = seq + 1

        # Re-encode payload using CBOR
        payload = cbor2.dumps(data)

        # Use adaptive QoS if available
        if qos is None:
            if self.qos_manager is not None:
                qos = self.qos_manager.getQoS()
            else:
                qos = 0
                rospy.loginfo("No qos_manager available; defaulting QoS to 0")

        # Publish to broker
        result = self.client.publish(topic, payload, qos=qos)

        # Track metrics if publish was successful
        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            if self.metrics:
                self.metrics.publish_timestamps[result.mid] = rospy.Time.now().to_sec()
                overhead = 4
                total_bytes = len(topic.encode('utf-8')) + len(payload) + overhead
                self.metrics.mqtt_total_bytes += total_bytes

            self.published_message_count += 1

            # Update per-topic publish counter
            topic = topic.strip()
            if topic not in self.published_message_counts:
                self.published_message_counts[topic] = 0
            self.published_message_counts[topic] += 1

        return result

    def publish_summary(self, qos=2):
        """
        Publishes a message containing per-topic counts and sequence numbers
        to a fixed summary topic. Used for monitoring and diagnostics.
        """
        summary_data = {
            "timestamp": rospy.Time.now().to_sec(),
            "topics": {}
        }

        # Limit summary to these topics
        allowed_topics = {"robot/battery_status", "robot/imu", "robot/odom", "robot/scan"}

        # Add topic stats to summary
        for topic, count in self.published_message_counts.items():
            if topic in allowed_topics:
                last_seq = self.sequence_numbers.get(topic, 0) - 1
                summary_data["topics"][topic] = {
                    "published_count": count,
                    "last_sequence": last_seq
                }

        payload = cbor2.dumps(summary_data)
        result = self.client.publish("mqtt/summary", payload, qos=qos)

        # Reset counters after publishing
        for topic in self.published_message_counts:
            if topic in allowed_topics:
                self.published_message_counts[topic] = 0
        self.published_message_count = 0

        return result

    def on_publish(self, client, userdata, mid):
        # Track publish latency and throughput if metrics are enabled
        if self.metrics:
            if mid in self.metrics.publish_timestamps:
                publish_time = self.metrics.publish_timestamps.pop(mid)
                ack_time = rospy.Time.now().to_sec()
                latency = ack_time - publish_time
                self.metrics.mqtt_publish_latencies.append(latency)

            if "published" not in self.metrics.throughput_counters:
                self.metrics.throughput_counters["published"] = 0
            self.metrics.throughput_counters["published"] += 1

    def disconnect(self):
        # Cleanly stop MQTT loop and disconnect
        self.client.loop_stop()
        self.client.disconnect()


# Optional: allows this module to run standalone for debugging
if __name__ == '__main__':
    rospy.init_node('mqtt_handler_node', anonymous=True)
    rospy.loginfo("Starting MQTT Handler Node")

    from metrics_manager import MetricsManager
    metrics = MetricsManager()

    # Minimal topic config for test mode
    topics = {
        "subscribe": [MQTT_RAW_COMMAND],
        "publish": ["robot/battery", "robot/battery_status", "mqtt/summary"]
    }

    mqtt_handler = MQTTHandler(topics=topics, metrics=metrics)
    mqtt_handler.connect()

    # Publish summary every 5 seconds
    def summary_timer_callback(event):
        mqtt_handler.publish_summary(qos=2)

    rospy.Timer(rospy.Duration(5.0), summary_timer_callback)

    rospy.spin()
    mqtt_handler.disconnect()
