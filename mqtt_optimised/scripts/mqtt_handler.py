# mqtt_handler.py
import paho.mqtt.client as mqtt
import json, cbor2, rospy
import logging
import hashlib
from config import MQTT_BROKER, MQTT_PORT, MQTT_RAW_COMMAND

class MQTTHandler:
    def __init__(self, broker=MQTT_BROKER, port=MQTT_PORT, topics=None, movement_controller=None, metrics=None, qos = None):
        self.broker = broker
        self.port = port
        self.topics = topics  # A dict of topics to subscribe/publish to.
        self.movement_controller = movement_controller
        self.metrics = metrics
        self.qos_manager = qos
        self.client = mqtt.Client()
        self._setup_callbacks()
        
        # Initialize sequence_numbers for publish topics.
        # Assuming self.topics["publish"] is a list of topics you'll publish to.
        if topics and "publish" in topics:
            self.sequence_numbers = {topic: 0 for topic in topics["publish"]}
        else:
            self.sequence_numbers = {}
    
    def _setup_callbacks(self):
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        self.client.on_publish = self.on_publish
    
    def on_connect(self, client, userdata, flags, rc):
        logging.info("Connected with result code %s", rc)
        for topic in self.topics.get("subscribe", []):
            client.subscribe(topic)
    
    def on_message(self, client, userdata, msg):
        logging.info("Received message on topic %s: %s", msg.topic, msg.payload.decode())
        print("HELLO")
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
                # Call the movement controller's process_command method.
                if self.movement_controller:
                    print("2")
                    self.movement_controller.process_command(command_data)
                else:
                    logging.error("Movement controller is not set!")
                    print("3 - erorr")
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

        # Inject publish_time if needed.
        if add_publish_time:
            data["publish_time"] = rospy.Time.now().to_sec()

        # Inject sequence number if needed.
        if add_sequence:
            seq = self.sequence_numbers.get(topic, 0)
            data["seq"] = seq
            self.sequence_numbers[topic] = seq + 1

        # Serialize the updated data back into payload.
        payload = cbor2.dumps(data)

        # Determine QoS if not provided.
        if qos is None:
            if self.qos_manager is not None:
                qos = self.qos_manager.getQoS()
            else:
                qos = 0
                rospy.loginfo("No qos_manager available; defaulting QoS to 0")
        
        # Publish the message.
        result = self.client.publish(topic, payload, qos=qos)
        
        if result.rc == mqtt.MQTT_ERR_SUCCESS and self.metrics:
            self.metrics.publish_timestamps[result.mid] = rospy.Time.now().to_sec()
            overhead = 4
            total_bytes = len(topic.encode('utf-8')) + len(payload) + overhead
            self.metrics.mqtt_total_bytes += total_bytes
        return result


    
    def on_publish(self, client, userdata, mid):
        if self.metrics:
            # If a publish timestamp exists for this message, calculate latency.
            if mid in self.metrics.publish_timestamps:
                publish_time = self.metrics.publish_timestamps.pop(mid)
                ack_time = rospy.Time.now().to_sec()
                latency = ack_time - publish_time
                self.metrics.mqtt_publish_latencies.append(latency)
                # rospy.loginfo("Message %d published. Latency: %.3f seconds", mid, latency)
            
            # Optional: Update a published messages counter if desired.
            # For example, you could add a 'published' counter in your throughput_counters.
            if "published" not in self.metrics.throughput_counters:
                self.metrics.throughput_counters["published"] = 0
            self.metrics.throughput_counters["published"] += 1
        
        # Optionally, if you want to update bandwidth here (if not done during publish),
        # you could add code here to update the total bytes published.
    
    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()
