#!/usr/bin/env python3
import json
import cbor2
import statistics
import rospy
import psutil
import threading
import time

class MetricsManager:
    def __init__(self):
        self.latest_battery_percentage = None 
        # Counters for messages received/published.
        self.throughput_counters = {
            "battery": 0,
            "odom": 0,
            "scan": 0,
            "imu": 0,
            "movement": 0,
        }
        # Records for latency per topic.
        self.latency_records = {
            "battery": [],
            "odom": [],
            "scan": [],
            "imu": [],
            "movement": 0,
        }
        # Error counters per topic.
        self.error_counters = {
            "battery": 0,
            "odom": 0,
            "scan": 0,
            "imu": 0,
            "movement": 0,
        }
        # Bandwidth counters in bytes.
        self.bandwidth_counters = {
            "battery": 0,
            "odom": 0,
            "scan": 0,
            "imu": 0,
            "movement": 0,
        }
        # Processing times (in seconds) for callbacks.
        self.processing_records = {
            "battery": [],
            "odom": [],
            "scan": [],
            "imu": [],
            "movement": [],
        }
        # Dictionary for MQTT publish timestamps.
        self.publish_timestamps = {}
        # List to record MQTT publish round-trip latencies.
        self.mqtt_publish_latencies = []
        # MQTT error counter.
        self.mqtt_errors = 0
        # Total bytes published.
        self.mqtt_total_bytes = 0
        self.mqtt_overall_bytes = 0
        # Duplicate messages count.
        self.duplicate_count = 0
        # Dictionary for message hash counts.
        self.message_hash_counts = {}
        # List to store QoS decisions.
        self.qos_decision_log = []
        self.current_qos = 0
        # Lock for synchronizing QoS updates.
        self.qos_lock = threading.Lock()

    # Update functions for counters.
    def update_throughput(self, topic, count=1):
        if topic in self.throughput_counters:
            self.throughput_counters[topic] += count

    def record_latency(self, topic, latency):
        if topic in self.latency_records:
            self.latency_records[topic].append(latency)

    def update_bandwidth(self, topic, payload_size):
        if topic in self.bandwidth_counters:
            self.bandwidth_counters[topic] += payload_size

    def record_processing_time(self, topic, processing_time):
        if topic in self.processing_records:
            self.processing_records[topic].append(processing_time)

    # Calculation functions.
    def calc_mqtt_latency(self):
        latencies = self.mqtt_publish_latencies.copy()
        if latencies:
            avg_latency = sum(latencies) / len(latencies)
            jitter = statistics.stdev(latencies) if len(latencies) > 1 else 0.0
            return avg_latency, jitter, len(latencies)
        return 0, 0, 0

    def calc_mqtt_bandwidth(self):
        bandwidth_usage = self.mqtt_total_bytes
        total_bytes = self.mqtt_total_bytes + self.mqtt_overall_bytes
        return bandwidth_usage, total_bytes

    def calc_mqtt_error_metrics(self):
        total_attempts = len(self.mqtt_publish_latencies) + self.mqtt_errors
        if total_attempts > 0:
            error_rate = (self.mqtt_errors / total_attempts) * 100.0
        else:
            error_rate = 0.0
        return error_rate, self.mqtt_errors, total_attempts

    def calc_mqtt_throughput(self, interval):
        count = len(self.mqtt_publish_latencies)
        throughput = count / interval if interval > 0 else 0.0
        return throughput

    def calc_mqtt_duplicate_count(self):
        total_messages = sum(self.message_hash_counts.values())
        if total_messages > 0:
            duplicate_rate = (self.duplicate_count / total_messages) * 100.0
            # print("duplicate rate is: ", duplicate_rate)
            # print("duplicate count is: ", self.duplicate_count)
            # print("total messages is: ", total_messages)
        else:
            duplicate_rate = 0.0
        return self.duplicate_count, duplicate_rate

    def calc_avg_payload_size(self):
        count = len(self.mqtt_publish_latencies)
        return self.mqtt_total_bytes / count if count > 0 else 0

    # Resource utilization functions.
    def get_cpu_usage(self):
        return psutil.cpu_percent(interval=None)

    def get_memory_usage(self):
        mem = psutil.virtual_memory()
        return mem.percent

    def getBatteryLevel(self):
        # Return the current battery level if available; otherwise, use a default value.
        if self.latest_battery_percentage is not None:
            return self.latest_battery_percentage
        return 1.0  # Or some default value (e.g., 100%)

    # Logging functions.
    def log_resource_utilization(self, event):
        cpu_usage = self.get_cpu_usage()
        memory_usage = self.get_memory_usage()
        battery_level = self.getBatteryLevel()
        if battery_level is None:
            battery_level = 1.0
        rospy.loginfo("Resource Utilization: CPU: %.1f%%, Memory: %.1f%%, Battery: %.1f%%",
                      cpu_usage, memory_usage, battery_level * 100)

    def log_overall_metrics(self, event):
        # Calculate time interval for this logging period.
        interval = event.current_real.to_sec() - (event.last_real.to_sec() if event.last_real else event.current_real.to_sec() - 1.0)
        print("interval is: ", interval)

        avg_latency, jitter, latency_count = self.calc_mqtt_latency()
        bandwidth_usage, total_bytes = self.calc_mqtt_bandwidth()
        error_rate, errors, attempts = self.calc_mqtt_error_metrics()
        throughput = self.calc_mqtt_throughput(interval)
        dupe_count, dupe_rate = self.calc_mqtt_duplicate_count()
        avg_payload_size = self.calc_avg_payload_size()

        rospy.loginfo("----- MQTT Overall Metrics (Every 5 seconds) -----")
        rospy.loginfo("Total Messages Published: %d", latency_count)
        rospy.loginfo("Total Throughput: %.2f messages/s", throughput)
        rospy.loginfo("Latency AVG: %.3f ms", avg_latency * 1000)
        rospy.loginfo("Jitter Rate AVG: %.3f ms", jitter * 1000)
        rospy.loginfo("Total Bandwidth: %d bytes", total_bytes)
        rospy.loginfo("Bandwidth Rate: %d bytes/s", int(bandwidth_usage / interval))
        rospy.loginfo("Average Payload Size: %d bytes", avg_payload_size)
        rospy.loginfo("Error Rate: %.0f%% (%d errors out of %d attempts)", error_rate, errors, attempts)
        rospy.loginfo("Duplicate Count: %d", dupe_count)
        rospy.loginfo("Duplicate Rate: %.0f%%", dupe_rate)
        rospy.loginfo("--------------------------------")

        # Reset metrics after logging.
        self.mqtt_publish_latencies.clear()
        self.mqtt_total_bytes = 0
        self.mqtt_errors = 0
        self.duplicate_count = 0

    def start_timers(self):
        # rospy.Timer(rospy.Duration(5.0), self.log_overall_metrics)
        rospy.Timer(rospy.Duration(5.0), self.log_resource_utilization)

# For convenience, if you want to use a global instance:
if __name__ == '__main__':
    rospy.init_node('metrics_manager_node', anonymous=True)
    mm = MetricsManager()
    mm.start_timers()
    rospy.spin()
