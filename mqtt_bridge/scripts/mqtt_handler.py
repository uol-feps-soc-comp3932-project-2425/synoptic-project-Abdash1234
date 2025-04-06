# mqtt_handler.py
import paho.mqtt.client as mqtt
import json, rospy
import logging
import hashlib
from config import MQTT_BROKER, MQTT_PORT, MQTT_RAW_COMMAND

class MQTTHandler:
    def __init__(self, broker=MQTT_BROKER, port=MQTT_PORT, topics=None, movement_controller=None):
        self.broker = broker
        self.port = port
        self.topics = topics  # A dict of topics to subscribe/publish to.
        self.movement_controller = movement_controller
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
    
    def on_connect(self, client, userdata, flags, rc):
        logging.info("Connected with result code %s", rc)
        for topic in self.topics.get("subscribe", []):
            client.subscribe(topic)
    
    def on_message(self, client, userdata, msg):
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
            data = json.loads(payload)
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
        payload = json.dumps(data)
        
        # Publish the message.
        self.client.publish(topic, payload, qos=0)



    
    # def on_publish(self, client, userdata, mid):
    #    rospy.loginfo("Message published with mid: %s", mid)
    
    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()
