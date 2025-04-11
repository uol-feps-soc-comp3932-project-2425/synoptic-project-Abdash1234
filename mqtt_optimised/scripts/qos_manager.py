#!/usr/bin/env python3
import threading
import rospy
import paho.mqtt.client as mqtt
import json
import csv
import os
from config import MQTT_BROKER, MQTT_PORT

# Define the CSV file path for logging QoS changes.
CSV_QOS_LOG_FILE = "/home/abdullah/catkin_ws/src/synoptic-project-Abdash1234/csv_files/qos_metrics.csv"

class QoSManager:
    def __init__(self, metrics_manager, smoothing_window=5):
        """
        Initialize with a reference to a MetricsManager instance.
        """
        self.metrics = metrics_manager
        self.current_qos = 0  # Default QoS
        self.previous_qos = self.current_qos  # To track changes.
        self.lock = threading.Lock()
        self.smoothing_window = smoothing_window
        self.score_history = []  # To store recent score values
        self.lower_threshold = 0.25  # Score must drop below this to decrease QoS.
        self.upper_threshold = 0.65  # Score must exceed this to increase QoS.

        self.command_loss_threshold = 0.5
        self.metrics_loss_threshold = 0.75
        self.latest_loss_pct = 0.0
        self.latest_loss_pct_metrics = 0.0

        # Set up an MQTT client to subscribe to the summary topic.
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.client.loop_start()  # Start the loop to listen for summary messages
        
        # Initialize CSV header if file doesn't exist.
        if not os.path.exists(CSV_QOS_LOG_FILE) or os.path.getsize(CSV_QOS_LOG_FILE) == 0:
            with open(CSV_QOS_LOG_FILE, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["timestamp", "new_qos", "smoothed_score", "battery_level", "avg_latency", "bandwidth_usage", "jitter", "error_rate"])

    def on_connect(self, client, userdata, flags, rc):
            rospy.loginfo("QoSManager connected to MQTT Broker with result code: %s", rc)
            client.subscribe("robot/command_ack")
            client.subscribe("mqtt/summary-metrics")


    def on_message(self, client, userdata, msg):
        try:
            # Check the topic of the incoming message.
            if msg.topic == "robot/command_ack":
                # Attempt to decode the summary message as JSON.
                summary_data = json.loads(msg.payload.decode('utf-8'))
                # For example, assume the summary message has a key "loss_pct"
                self.latest_loss_pct = summary_data.get("loss_pct", 0.0)
                # print("message recieved", self.latest_loss_pct)
                # Adjust QoS based on the latest loss percentage.
                self.adjust_qos()
            elif msg.topic == "mqtt/summary-metrics":
                # Process the summary metrics message.
                summary_data = json.loads(msg.payload.decode('utf-8'))
                self.latest_loss_pct_metrics = summary_data.get("error_rate_pct", 0.0)
                # rospy.loginfo("Received summary metrics: %s", summary_data)
                print("this is the latest loss metric",self.latest_loss_pct_metrics)
        except Exception as e:
            rospy.logerr("Error processing summary message: %s", e)

    def shutdown(self):
        self.client.loop_stop()
        self.client.disconnect()


    def normaliseLatency(self, latency, minLatency=0.001, maxLatency=0.1):
        """
        Normalize latency to a value between 0 and 1.
        """
        if latency <= minLatency:
            return 0.0
        elif latency >= maxLatency:
            return 1.0
        else:
            return (latency - minLatency) / (maxLatency - minLatency)

    def normaliseBandwidth(self, bandwidth, low_bw=1000, high_bw=100000):
        """
        Normalize bandwidth to a value between 0 and 1.
        """
        if bandwidth <= low_bw:
            return 0.0
        elif bandwidth >= high_bw:
            return 1.0
        else:
            return (bandwidth - low_bw) / (high_bw - low_bw)

    def normaliseJitter(self, jitter, minJitter=0, maxJitter=0.005):
        """
        Normalize jitter to a value between 0 and 1.
        """
        if jitter <= minJitter:
            return 0.0
        elif jitter >= maxJitter:
            return 1.0
        else:
            return (jitter - minJitter) / (maxJitter - minJitter)

    def calcScore(self, battery_level, latency, bandwidth, jitter):
        """
        Calculate a weighted score based on the input metrics.
        Lower score means conditions are good, higher score indicates worse conditions.
        All metrics are normalized to the range [0, 1] before applying weights.
        """
        # Battery level is assumed to be a fraction between 0 and 1.
        norm_battery = 1.0 - battery_level  # Lower battery means worse condition

        norm_latency = self.normaliseLatency(latency)
        norm_bandwidth = self.normaliseBandwidth(bandwidth)
        norm_jitter = self.normaliseJitter(jitter)

        # Weights for each metric; adjust these based on your system's requirements.
        w_battery = 0.2
        w_latency = 0.30
        w_bw      = 0.35
        w_jitter  = 0.15

        score = (w_battery * norm_battery +
                 w_latency * norm_latency +
                 w_bw      * norm_bandwidth +
                 w_jitter  * norm_jitter)
        return score

    def log_qos_change(self, new_qos, smoothed_score, battery_level, avg_latency, bandwidth_usage, jitter, error_rate):
        """
        Log a QoS change event to a CSV file.
        """
        timestamp = rospy.get_time()
        row = [
            timestamp,
            new_qos,
            smoothed_score,
            battery_level,
            avg_latency,
            bandwidth_usage,
            jitter,
            error_rate
        ]
        try:
            with open(CSV_QOS_LOG_FILE, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(row)
            # rospy.loginfo("Logged QoS change: QoS %d at %.3f (battery: %.2f, latency: %.3f, bw: %.2f, jitter: %.3f, error: %.2f%%)", new_qos, smoothed_score, battery_level, avg_latency, bandwidth_usage, jitter, error_rate)
        except Exception as e:
            rospy.logerr("Error logging QoS change: %s", e)

    def setQoS(self):
        """
        Calculate the score from current metrics, smooth it over a window,
        and update the QoS level only if the smoothed score exceeds hysteresis thresholds.
        Logs the change when it occurs.
        """
        self.current_qos = 0
        battery_level = self.metrics.getBatteryLevel()
        avg_latency, jitter, _ = self.metrics.calc_mqtt_latency()
        bandwidth_usage, _ = self.metrics.calc_mqtt_bandwidth()
        error_rate = 0
        if self.latest_loss_pct_metrics > self.latest_loss_pct:
            error_rate = self.latest_loss_pct_metrics
        else:
            error_rate = self.latest_loss_pct

        # Calculate current score.
        current_score = self.calcScore(battery_level, avg_latency, bandwidth_usage, jitter)
        self.score_history.append(current_score)
        if len(self.score_history) > self.smoothing_window:
            self.score_history.pop(0)
        smoothed_score = sum(self.score_history) / len(self.score_history)

        if self.latest_loss_pct > self.command_loss_threshold:
            print("Command Loss threshold exceed: QoS -> Lvl 2")
            self.current_qos = 2
            self.log_qos_change(2, smoothed_score, battery_level, avg_latency, bandwidth_usage, jitter, error_rate)
            return
        if self.latest_loss_pct_metrics > self.metrics_loss_threshold:
            print("Metrics Loss threshold exceed: QoS -> Lvl 2")
            self.current_qos = 2
            self.log_qos_change(2, smoothed_score, battery_level, avg_latency, bandwidth_usage, jitter, error_rate)
            return

        with self.lock:
            new_qos = self.current_qos
            # Hysteresis logic based on current QoS level:
            if self.current_qos == 0:
                # If conditions worsen significantly, move to QoS 1.
                if smoothed_score >= self.upper_threshold:
                    new_qos = 1
            elif self.current_qos == 1:
                # If conditions improve, drop back to QoS 0.
                if smoothed_score < self.lower_threshold:
                    new_qos = 0
                # If conditions worsen further, move to QoS 2.
                elif smoothed_score >= self.upper_threshold:
                    new_qos = 2
            elif self.current_qos == 2:
                # If conditions improve sufficiently, drop to QoS 1.
                if smoothed_score < self.upper_threshold:
                    new_qos = 1

            # If the QoS level has changed, log the event.
            if new_qos != self.current_qos:
                print("Old QoS: ", self.current_qos)
                print("New QoS: ", new_qos)
                self.current_qos = new_qos
                self.log_qos_change(new_qos, smoothed_score, battery_level, avg_latency, bandwidth_usage, jitter, error_rate)

        rospy.loginfo("QoSManager: Current QoS is %d (smoothed score: %.3f)", self.current_qos, smoothed_score)
        return self.current_qos
    
    def increase_qos(self):
        """Increase QoS by 1, up to a max of 2."""
        with self.lock:
            if self.current_qos < 2:
                self.current_qos += 1
                # rospy.loginfo("QoS increased to %d", self.current_qos)
        return self.current_qos

    def getQoS(self):
        """
        Return the current QoS level in a thread-safe manner.
        """
        with self.lock:
            return self.current_qos if self.current_qos is not None else 0

    def start(self):
        """
        Start the dynamic QoS updater loop.
        This loop updates the QoS every 2 seconds.
        """
        rate = rospy.Rate(0.5)  # 0.5 Hz means every 2 seconds.
        while not rospy.is_shutdown():
            self.setQoS()
            rate.sleep()

if __name__ == '__main__':
    # print("Starting QoSManager...")
    qos_manager = QoSManager(loss_threshold=5.0)
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        qos_manager.shutdown()
