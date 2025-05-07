# aggregator.py
import rospy
import cbor2
import threading

class Aggregator:
        # Aggregates multiple messages over a time interval and publishes them as a single MQTT message. Helps reduce bandwidth.

    def __init__(self, mqtt_handler, topic, flush_interval=5.0):

        #Initialize the aggregator.
        self.mqtt_handler = mqtt_handler
        self.topic = topic
        self.flush_interval = flush_interval
        self.buffer = []
        self.lock = threading.Lock()
        # Use rospy.Timer to call flush periodically
        self.timer = rospy.Timer(rospy.Duration(self.flush_interval), self._timer_callback)
        # print("Aggregator for topic '%s' started with flush_interval=%.2fs", self.topic, self.flush_interval)

    def _timer_callback(self, event):
        # Timer callback triggered by ROS every flush_interval seconds.
        # Calls the flush method to publish buffered messages.
        # print("Aggregator timer callback triggered")
        self.flush()

    def add_message(self, payload):
        # adds a new message to the buffer 
        with self.lock:
            self.buffer.append(payload)
            # print("Aggregator added message. Buffer size is now %d", len(self.buffer))

    def flush(self):
        # Publish all buffered messages as one aggregated message and clears the buffer
        with self.lock:
            if not self.buffer:
                print("Aggregator flush called but buffer is empty.")
                return
            aggregated_payload = cbor2.dumps(self.buffer)
            # print("Publishing aggregated message on topic '%s' with %d messages", self.topic, len(self.buffer))
            # print("Publishing aggregated payload:", aggregated_payload)
            # print("This is the length of the payload: %d", len(self.buffer))
            self.mqtt_handler.publish(self.topic, aggregated_payload)
            self.buffer.clear()

    def stop(self):
        # Stop the ROS timer to cleanly shut down the aggregator.

        self.timer.shutdown()
        rospy.loginfo("Aggregator for topic '%s' stopped", self.topic)
