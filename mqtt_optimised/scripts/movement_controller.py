#!/usr/bin/env python3

import json
import math
import rospy
import tf
import cbor2
import time
from geometry_msgs.msg import Twist
from config import MQTT_OPTIMISED_COMMAND

class MovementController:
    def __init__(self, mqtt_handler, metrics=None):
        # Save MQTT handler instance for publishing commands
        self.mqtt_handler = mqtt_handler
        # Optional: pass in a MetricsManager to track throughput, latency, etc.
        self.metrics = metrics

        # State variables to track turning behaviour
        self.turning = False
        self.initial_theta = None
        self.current_turn_threshold = None

    def process_command(self, command_data):
        """
        Process a movement command received via MQTT.
        Converts command into a ROS Twist message and publishes it.
        Tracks latency, throughput, and bandwidth if metrics are enabled.
        """
        start_time = rospy.Time.now().to_sec()

        # Optional latency tracking if timestamp is provided in message
        timestamp = command_data.get("timestamp")
        seq = command_data.get("seq")

        if timestamp:
            cmd_latency = start_time - timestamp
            rospy.loginfo("End-to-End Command Latency (MovementController): %.3f seconds", cmd_latency)

        # Update movement command throughput counter if metrics are enabled
        if self.metrics:
            if "movement" not in self.metrics.throughput_counters:
                self.metrics.throughput_counters["movement"] = 0
            self.metrics.update_throughput("movement", 1)

        # Create a new Twist message for robot movement
        twist = Twist()
        cmd = command_data.get("command", "")
        speed_val = command_data.get("speed", 2)
        rospy.loginfo("MovementController: Received command %s", cmd)

        # Map commands to twist motions
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
            twist.angular.z = -0.5  # Right turn (clockwise)
            self.current_turn_threshold = math.pi / 2  # 90 degrees
            self.turning = True
            self.initial_theta = None  # Set later from odometry
            rospy.loginfo("Executing turn_right_90 command")
            self.publish_twist(twist, timestamp, seq)

        elif cmd == "turn_left_90":
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # Left turn (counterclockwise)
            self.current_turn_threshold = math.pi / 2
            self.turning = True
            self.initial_theta = None
            rospy.loginfo("Executing turn_left_90 command")
            self.publish_twist(twist, timestamp, seq)

        elif cmd == "rotate":
            twist.linear.x = 0.0
            twist.angular.z = 0.5  # 180-degree spin
            self.current_turn_threshold = math.pi
            self.turning = True
            self.initial_theta = None
            rospy.loginfo("Executing rotate command")
            self.publish_twist(twist, timestamp, seq)

        else:
            rospy.logwarn("Unknown movement command received: %s", cmd)

        # Record how long this processing took
        finish_time = rospy.Time.now().to_sec()
        processing_time = finish_time - start_time
        rospy.loginfo("MovementController: Processing time: %.3f seconds", processing_time)

        # Record processing time and estimate bandwidth usage
        if self.metrics:
            self.metrics.record_processing_time("movement", processing_time)

            # Prepare payload for size estimation
            twist_dict = {
                "linear": {"x": twist.linear.x, "y": twist.linear.y, "z": twist.linear.z},
                "angular": {"x": twist.angular.x, "y": twist.angular.y, "z": twist.angular.z}
            }
            if timestamp:
                twist_dict["timestamp"] = timestamp
            if seq is not None:
                twist_dict["seq"] = seq

            payload = cbor2.dumps(twist_dict)
            overhead = 4  # MQTT overhead bytes
            total_bytes = len(MQTT_OPTIMISED_COMMAND.encode('utf-8')) + len(payload) + overhead
            self.metrics.update_bandwidth("movement", total_bytes)

    def odom_turning_callback(self, odom_msg):
        """
        Monitors odometry during a turn. Once the robot has turned
        the desired amount, sends a stop command automatically.
        """
        if not self.turning:
            return

        # Extract yaw angle (theta) from odometry quaternion
        q = odom_msg.pose.pose.orientation
        (_, _, current_theta) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])

        if self.initial_theta is None:
            # Store starting angle at beginning of turn
            self.initial_theta = current_theta
            rospy.loginfo("Recorded initial orientation: %.3f radians", self.initial_theta)
        else:
            # Check if the turn has reached the target angle
            if self._has_turned(self.initial_theta, current_theta, self.current_turn_threshold):
                rospy.loginfo("Turn complete: initial=%.3f, current=%.3f (threshold=%.3f)",
                              self.initial_theta, current_theta, self.current_turn_threshold)

                # Stop the robot once turn is complete
                stop_twist = Twist()
                stop_twist.linear.x = 0.0
                stop_twist.angular.z = 0.0
                self.publish_twist(stop_twist)

                # Reset turning state
                self.turning = False
                self.initial_theta = None
                self.current_turn_threshold = None

    def _has_turned(self, initial, current, threshold):
        """
        Compute angular difference and check if turn threshold is met.
        Handles angle wraparound from -π to π.
        """
        diff = current - initial
        diff = (diff + math.pi) % (2 * math.pi) - math.pi  # Normalize to [-π, π]
        return abs(diff) >= threshold

    def publish_twist(self, twist, timestamp=None, seq=None):
        """
        Publish a Twist message over MQTT using CBOR encoding.
        Includes optional timestamp and sequence number.
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
