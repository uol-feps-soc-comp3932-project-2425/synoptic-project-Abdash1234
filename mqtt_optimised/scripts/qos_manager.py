#!/usr/bin/env python3
import threading
import rospy
import statistics
import csv
import os

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
        
        # Initialize CSV header if file doesn't exist.
        if not os.path.exists(CSV_QOS_LOG_FILE) or os.path.getsize(CSV_QOS_LOG_FILE) == 0:
            with open(CSV_QOS_LOG_FILE, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(["timestamp", "new_qos", "smoothed_score", "battery_level", "avg_latency", "bandwidth_usage", "jitter", "error_rate"])

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

    def calcScore(self, battery_level, latency, bandwidth, jitter, error_rate):
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
        norm_error_rate = error_rate / 100.0  # Convert percentage to fraction

        # Weights for each metric; adjust these based on your system's requirements.
        w_battery = 0.3
        w_latency = 0.35
        w_error   = 0.2
        w_bw      = 0.1
        w_jitter  = 0.05

        score = (w_battery * norm_battery +
                 w_latency * norm_latency +
                 w_error   * norm_error_rate +
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
            rospy.loginfo("Logged QoS change: QoS %d at %.3f (battery: %.2f, latency: %.3f, bw: %.2f, jitter: %.3f, error: %.2f%%)", 
                          new_qos, smoothed_score, battery_level, avg_latency, bandwidth_usage, jitter, error_rate)
        except Exception as e:
            rospy.logerr("Error logging QoS change: %s", e)

    def setQoS(self):
        """
        Calculate the score from current metrics, smooth it over a window,
        and update the QoS level only if the smoothed score exceeds hysteresis thresholds.
        Logs the change when it occurs.
        """
        battery_level = self.metrics.getBatteryLevel()
        avg_latency, jitter, _ = self.metrics.calc_mqtt_latency()
        bandwidth_usage, _ = self.metrics.calc_mqtt_bandwidth()
        error_rate, _, _ = self.metrics.calc_mqtt_error_metrics()

        # Calculate current score.
        current_score = self.calcScore(battery_level, avg_latency, bandwidth_usage, jitter, error_rate)
        self.score_history.append(current_score)
        if len(self.score_history) > self.smoothing_window:
            self.score_history.pop(0)
        smoothed_score = sum(self.score_history) / len(self.score_history)

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
                self.current_qos = new_qos
                self.log_qos_change(new_qos, smoothed_score, battery_level, avg_latency, bandwidth_usage, jitter, error_rate)

        rospy.loginfo("QoSManager: Current QoS is %d (smoothed score: %.3f)", self.current_qos, smoothed_score)
        return self.current_qos
    
    def increase_qos(self):
        """Increase QoS by 1, up to a max of 2."""
        with self.lock:
            if self.current_qos < 2:
                self.current_qos += 1
                rospy.loginfo("QoS increased to %d", self.current_qos)
            else:
                rospy.loginfo("QoS remains at %d", self.current_qos)
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
    rospy.init_node('qos_manager_node', anonymous=True)
    
    from metrics_manager import MetricsManager
    metrics = MetricsManager()
    qos_manager = QoSManager(metrics)
    qos_manager.start()
