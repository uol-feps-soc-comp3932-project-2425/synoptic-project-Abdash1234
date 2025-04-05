# aggregator.py
import rospy
import cbor2
import threading

class Aggregator:
    def __init__(self, mqtt_handler, topic, flush_interval=5.0):
        """
        mqtt_handler: An instance of MQTTHandler to publish messages.
        topic: The MQTT topic to publish aggregated messages to.
        flush_interval: Time interval (in seconds) to flush the buffer.
        """
        self.mqtt_handler = mqtt_handler
        self.topic = topic
        self.flush_interval = flush_interval
        self.buffer = []
        self.lock = threading.Lock()
        # Use rospy.Timer to call flush periodically
        self.timer = rospy.Timer(rospy.Duration(self.flush_interval), self._timer_callback)
        # print("Aggregator for topic '%s' started with flush_interval=%.2fs", self.topic, self.flush_interval)

    def _timer_callback(self, event):
        print("Aggregator timer callback triggered")
        self.flush()

    def add_message(self, payload):
        """
        Add a new message payload (assumed to be already serialized, e.g., via cbor2) to the buffer.
        """
        with self.lock:
            self.buffer.append(payload)
            # print("Aggregator added message. Buffer size is now %d", len(self.buffer))

    def flush(self):
        """
        Flush the current buffer by aggregating the messages and publishing.
        """
        with self.lock:
            if not self.buffer:
                print("Aggregator flush called but buffer is empty.")
                return
            aggregated_payload = cbor2.dumps(self.buffer)
            # print("Publishing aggregated message on topic '%s' with %d messages", self.topic, len(self.buffer))
            # print("Publishing aggregated payload:", aggregated_payload)
            print("This is the length of the payload: %d", len(self.buffer))
            self.mqtt_handler.publish(self.topic, aggregated_payload)
            self.buffer.clear()

    def stop(self):
        """
        Stop the timer when shutting down.
        """
        self.timer.shutdown()
        rospy.loginfo("Aggregator for topic '%s' stopped", self.topic)
