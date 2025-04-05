#!/usr/bin/env python3
import rospy
import threading
import time

class QoSManager:
    def __init__(self, metrics_manager):
        """
        Initialize with a reference to a MetricsManager instance.
        """
        self.metrics = metrics_manager
        self.current_qos = 0  # Default QoS
        self.lock = threading.Lock()

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

    def setQoS(self):
        """
        Read the metrics from the MetricsManager, calculate a score, and decide on a QoS level.
        QoS 0 for good conditions, QoS 1 for moderate, QoS 2 for poor conditions.
        """
        # Read metrics from the metrics manager.
        battery_level = self.metrics.getBatteryLevel()
        avg_latency, jitter, _ = self.metrics.calc_mqtt_latency()
        bandwidth_usage, _ = self.metrics.calc_mqtt_bandwidth()
        error_rate, _, _ = self.metrics.calc_mqtt_error_metrics()

        # Calculate a score using normalized metric values.
        score = self.calcScore(battery_level, avg_latency, bandwidth_usage, jitter, error_rate)
        
        # Decide QoS based on the score thresholds.
        if score < 0.3:
            new_qos = 0
        elif score < 0.6:
            new_qos = 1
        else:
            new_qos = 2

        with self.lock:
            self.current_qos = new_qos

        rospy.loginfo("QoSManager: New QoS set: %d (score: %.3f)", new_qos, score)
        return new_qos
    
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
    # This block allows you to run the QoSManager independently for testing.
    rospy.init_node('qos_manager_node', anonymous=True)
    
    from metrics_manager import MetricsManager
    metrics = MetricsManager()
    qos_manager = QoSManager(metrics)
    qos_manager.start()
