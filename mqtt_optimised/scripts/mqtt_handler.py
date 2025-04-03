# mqtt_handler.py
import paho.mqtt.client as mqtt
import json, cbor2
import logging
from config import MQTT_BROKER, MQTT_PORT, MQTT_RAW_COMMAND

class MQTTHandler:
    def __init__(self, broker=MQTT_BROKER, port=MQTT_PORT, topics=None, movement_controller=None):
        self.broker = broker
        self.port = port
        self.topics = topics  # A dict of topics to subscribe/publish to
        self.movement_controller = movement_controller  # Dependency injection of movement controller
        self.client = mqtt.Client()
        self._setup_callbacks()
    
    def _setup_callbacks(self):
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
    
    def on_connect(self, client, userdata, flags, rc):
        logging.info("Connected with result code %s", rc)
        for topic in self.topics.get("subscribe", []):
            client.subscribe(topic)
    
    def on_message(self, client, userdata, msg):
        logging.info("Received message on topic %s: %s", msg.topic, msg.payload.decode())
        print("HELLO")
        # Check if the message is on the raw command topic.
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
    
    def publish(self, topic, payload, qos=0):
        result = self.client.publish(topic, payload, qos=qos)
        return result
    
    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()
