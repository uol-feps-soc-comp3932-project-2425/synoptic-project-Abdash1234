#!/usr/bin/env python3
import paho.mqtt.client as mqtt
import json, cbor2, rospy
import logging
import hashlib
from config import MQTT_BROKER, MQTT_PORT, MQTT_RAW_COMMAND

class MQTTHandler:
    def __init__(self, broker=MQTT_BROKER, port=MQTT_PORT, topics=None, movement_controller=None, metrics=None, qos=None):
        self.broker = broker
        self.port = port
        self.topics = topics  # A dict with "subscribe" and "publish" keys.
        self.movement_controller = movement_controller
        self.metrics = metrics
        self.qos_manager = qos
        self.client = mqtt.Client()
        self._setup_callbacks()
        
        # Initialize sequence_numbers and per-topic published counters for publish topics.
        if topics and "publish" in topics:
            self.sequence_numbers = {topic: 0 for topic in topics["publish"]}
            self.published_message_counts = {topic: 0 for topic in topics["publish"]}
        else:
            self.sequence_numbers = {}
            self.published_message_counts = {}
        
        # Also keep a global counter if needed.
        self.published_message_count = 0

    def _setup_callbacks(self):
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_publish = self.on_publish

    def on_connect(self, client, userdata, flags, rc):
        logging.info("Connected with result code %s", rc)
        for topic in self.topics.get("subscribe", []):
            client.subscribe(topic)
    
    def on_message(self, client, userdata, msg):
        # logging.info("Received message on topic %s: %s", msg.topic, msg.payload.decode())
        # Check if the message is on the raw command topic.
        if self.metrics:
            payload_hash = hashlib.sha256(msg.payload).hexdigest()
            if payload_hash in self.metrics.message_hash_counts:
                self.metrics.message_hash_counts[payload_hash] += 1
                # Every message beyond the first is considered a duplicate.
                self.metrics.duplicate_count += 1
            else:
                self.metrics.message_hash_counts[payload_hash] = 1

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
        self.client.connect(self.broker, self.port, 60)
        self.client.loop_start()

    def publish(self, topic, payload, qos=None, add_publish_time=True, add_sequence=True):
        # Decode payload into a dictionary.
        try:
            data = cbor2.loads(payload)
            if not isinstance(data, dict):
                data = {"payload": data}
        except Exception as e:
            rospy.logwarn("Failed to decode payload: %s. Wrapping raw payload.", e)
            data = {"payload": payload}

        if add_publish_time:
            data["publish_time"] = rospy.Time.now().to_sec()

        if add_sequence:
            seq = self.sequence_numbers.get(topic, 0)
            data["seq"] = seq
            self.sequence_numbers[topic] = seq + 1

        payload = cbor2.dumps(data)

        if qos is None:
            if self.qos_manager is not None:
                qos = self.qos_manager.getQoS()
            else:
                qos = 0
                rospy.loginfo("No qos_manager available; defaulting QoS to 0")

        result = self.client.publish(topic, payload, qos=qos)

        if result.rc == mqtt.MQTT_ERR_SUCCESS:
            if self.metrics:
                self.metrics.publish_timestamps[result.mid] = rospy.Time.now().to_sec()
                overhead = 4
                total_bytes = len(topic.encode('utf-8')) + len(payload) + overhead
                self.metrics.mqtt_total_bytes += total_bytes
            self.published_message_count += 1

            # Update or initialize the per-topic counter.
            topic = topic.strip()  # Ensure no extra whitespace.
            if topic not in self.published_message_counts:
                self.published_message_counts[topic] = 0
            self.published_message_counts[topic] += 1
            # print("Topic '%s' count updated to %d", topic, self.published_message_counts[topic])
        
        return result

    def publish_summary(self, qos=2):
        """
        Publish a summary message containing the per-topic published message counts,
        the last sequence number used for each topic, and a timestamp.
        This is published with QoS 2 for reliability.
        """
        rospy.loginfo("Publishing Summary")
        summary_data = {
            "timestamp": rospy.Time.now().to_sec(),
            "topics": {}
        }
        
        # Define which topics we want to include in the summary.
        allowed_topics = {"robot/battery_status", "robot/imu", "robot/odom", "robot/scan"}
        
        # Build summary for each publish topic if it is in allowed_topics.
        for topic, count in self.published_message_counts.items():
            if topic in allowed_topics:
                last_seq = self.sequence_numbers.get(topic, 0) - 1  # last published sequence number
                summary_data["topics"][topic] = {
                    "published_count": count,
                    "last_sequence": last_seq
                }
        payload = cbor2.dumps(summary_data)
        result = self.client.publish("mqtt/summary", payload, qos=qos)
        # rospy.loginfo("Published summary: %s", summary_data)
        # Reset the per-topic counters after publishing the summary.
        for topic in self.published_message_counts:
            if topic in allowed_topics:
                self.published_message_counts[topic] = 0
        self.published_message_count = 0
        return result


    def on_publish(self, client, userdata, mid):
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
        self.client.loop_stop()
        self.client.disconnect()

# For testing the MQTTHandler independently.
if __name__ == '__main__':
    rospy.init_node('mqtt_handler_node', anonymous=True)
    rospy.loginfo("Starting MQTT Handler Node")
    
    from metrics_manager import MetricsManager  # Ensure MetricsManager is defined and working.
    metrics = MetricsManager()
    # Example topics dictionary. Adjust as needed.
    topics = {
        "subscribe": [MQTT_RAW_COMMAND],
        "publish": ["robot/battery", "robot/battery_status", "mqtt/summary"]
    }
    mqtt_handler = MQTTHandler(topics=topics, metrics=metrics)
    mqtt_handler.connect()
    
    def summary_timer_callback(event):
        rospy.loginfo("Summary timer callback triggered")
        mqtt_handler.publish_summary(qos=2)
    
    rospy.Timer(rospy.Duration(5.0), summary_timer_callback)
    
    rospy.spin()
    mqtt_handler.disconnect()
