#!/usr/bin/env python3
import math
import rospy
import tf
import cbor2
import time
from geometry_msgs.msg import Twist
from config import MQTT_OPTIMISED_COMMAND

class MovementController:
    def __init__(self, mqtt_handler, metrics=None):
        self.mqtt_handler = mqtt_handler  # Save the MQTT handler instance.
        self.metrics = metrics            # Optional: MetricsManager instance.
        # State variables for turning.
        self.turning = False
        self.initial_theta = None
        self.current_turn_threshold = None

    def process_command(self, command_data):
        """
        Process movement commands from raw MQTT message data and publish
        a corresponding Twist command via MQTT. Also update movement metrics.
        """
        start_time = rospy.Time.now().to_sec()
        
        # Check for a timestamp in the command_data to measure end-to-end latency.
        timestamp = command_data.get("timestamp")
        # Extract the sequence number from the command.
        seq = command_data.get("seq")
        if timestamp:
            cmd_latency = start_time - timestamp
            rospy.loginfo("End-to-End Command Latency (MovementController): %.3f seconds", cmd_latency)
            # Optionally update a metric here if using a MetricsManager.

        # If metrics is available, update throughput for movement commands.
        if self.metrics:
            if "movement" not in self.metrics.throughput_counters:
                self.metrics.throughput_counters["movement"] = 0
            self.metrics.update_throughput("movement", 1)

        twist = Twist()
        cmd = command_data.get("command", "")
        speed_val = command_data.get("speed", 2)
        rospy.loginfo("MovementController: Received command %s", cmd)

        if cmd == "go_forward":
            twist.linear.x = speed_val
            twist.angular.z = 0.0
            rospy.loginfo("Executing go_forward command with speed: %s", speed_val)
            self.publish_twist(twist, timestamp, seq)
        elif cmd == "go_backwards":
            twist.linear.x = -speed_val
            twist.angular.z = 0.0
            rospy.loginfo("Executing go_backwards command with speed: %s", speed_val)
            self.publish_twist(twist, timestamp, seq)
        elif cmd == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            rospy.loginfo("Executing stop command")
            self.publish_twist(twist, timestamp, seq)
            self.turning = False
            self.initial_theta = None
            self.current_turn_threshold = None
        elif cmd == "turn_right_90":
            twist.linear.x = 0.0
            twist.angular.z = -0.5  # Negative angular for right turn.
            self.current_turn_threshold = math.pi / 2  # 90° turn.
            self.turning = True
            self.initial_theta = None  # Will be set in odom_turning_callback.
            rospy.loginfo("Executing turn_right_90 command")
            self.publish_twist(twist, timestamp, seq)
        elif cmd == "turn_left_90":
            twist.linear.x = 0.0
            twist.angular.z = 0.5   # Positive angular for left turn.
            self.current_turn_threshold = math.pi / 2  # 90° turn.
            self.turning = True
            self.initial_theta = None
            rospy.loginfo("Executing turn_left_90 command")
            self.publish_twist(twist, timestamp, seq)
        elif cmd == "rotate":
            twist.linear.x = 0.0
            twist.angular.z = 0.5   # Adjust as needed for rotation.
            self.current_turn_threshold = math.pi  # 180° turn.
            self.turning = True
            self.initial_theta = None
            rospy.loginfo("Executing rotate command")
            self.publish_twist(twist, timestamp, seq)
        else:
            rospy.logwarn("Unknown movement command received: %s", cmd)

        finish_time = rospy.Time.now().to_sec()
        processing_time = finish_time - start_time
        rospy.loginfo("MovementController: Processing time: %.3f seconds", processing_time)
        if self.metrics:
            self.metrics.record_processing_time("movement", processing_time)
            # Optionally update bandwidth for movement command.
            twist_dict = {
                "linear": {"x": twist.linear.x, "y": twist.linear.y, "z": twist.linear.z},
                "angular": {"x": twist.angular.x, "y": twist.angular.y, "z": twist.angular.z}
            }
            # Include the timestamp and sequence number in the payload if available.
            if timestamp:
                twist_dict["timestamp"] = timestamp
            if seq is not None:
                twist_dict["seq"] = seq
            payload = cbor2.dumps(twist_dict)
            overhead = 4
            total_bytes = len(MQTT_OPTIMISED_COMMAND.encode('utf-8')) + len(payload) + overhead
            self.metrics.update_bandwidth("movement", total_bytes)

    def odom_turning_callback(self, odom_msg):
        """
        Monitor odometry during a turn. When the required angular threshold is met,
        publish a stop command via MQTT.
        """
        if not self.turning:
            return
        
        # Extract current yaw from the odometry message.
        q = odom_msg.pose.pose.orientation
        (_, _, current_theta) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
        
        if self.initial_theta is None:
            self.initial_theta = current_theta
            rospy.loginfo("Recorded initial orientation: %.3f radians", self.initial_theta)
        else:
            if self._has_turned(self.initial_theta, current_theta, self.current_turn_threshold):
                rospy.loginfo("Turn complete: initial=%.3f, current=%.3f (threshold=%.3f)",
                              self.initial_theta, current_theta, self.current_turn_threshold)
                stop_twist = Twist()
                stop_twist.linear.x = 0.0
                stop_twist.angular.z = 0.0
                self.publish_twist(stop_twist)
                self.turning = False
                self.initial_theta = None
                self.current_turn_threshold = None

    def _has_turned(self, initial, current, threshold):
        diff = current - initial
        diff = (diff + math.pi) % (2 * math.pi) - math.pi
        return abs(diff) >= threshold

    def publish_twist(self, twist, timestamp=None, seq=None):
        """
        Publish a Twist command over MQTT.
        If a timestamp is provided, include it in the published payload.
        If a sequence number is provided, include it as well.
        """
        twist_dict = {
            "linear": {
                "x": twist.linear.x,
                "y": twist.linear.y,
                "z": twist.linear.z
            },
            "angular": {
                "x": twist.angular.x,
                "y": twist.angular.y,
                "z": twist.angular.z
            }
        }
        if timestamp:
            twist_dict["timestamp"] = timestamp
        if seq is not None:
            twist_dict["seq"] = seq
        payload = cbor2.dumps(twist_dict)
        self.mqtt_handler.publish(MQTT_OPTIMISED_COMMAND, payload)
