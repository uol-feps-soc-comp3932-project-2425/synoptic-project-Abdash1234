# movement_controller.py
#!/usr/bin/env python3
import math
import rospy
import tf
import cbor2
from geometry_msgs.msg import Twist
from config import MQTT_OPTIMISED_COMMAND

class MovementController:
    def __init__(self, mqtt_handler):
        self.mqtt_handler = mqtt_handler  # Save the MQTT handler instance.
        # State variables for turning.
        self.turning = False
        self.initial_theta = None
        self.current_turn_threshold = None

    def process_command(self, command_data):
        """
        Process movement commands from raw MQTT message data and publish
        a corresponding Twist command via MQTT.
        """
        twist = Twist()
        cmd = command_data.get("command", "")
        speed_val = command_data.get("speed", 2)
        rospy.loginfo("IN THE FUNC")
        print("IN THE FUNC")

        if cmd == "go_forward":
            twist.linear.x = speed_val
            twist.angular.z = 0.0
            rospy.loginfo("Executing go_forward command with speed: %s", speed_val)
            self.publish_twist(twist)
        elif cmd == "go_backwards":
            twist.linear.x = -speed_val
            twist.angular.z = 0.0
            rospy.loginfo("Executing go_backwards command with speed: %s", speed_val)
            self.publish_twist(twist)
        elif cmd == "stop":
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            rospy.loginfo("Executing stop command")
            self.publish_twist(twist)
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
            self.publish_twist(twist)
        elif cmd == "turn_left_90":
            twist.linear.x = 0.0
            twist.angular.z = 0.5   # Positive angular for left turn.
            self.current_turn_threshold = math.pi / 2  # 90° turn.
            self.turning = True
            self.initial_theta = None
            rospy.loginfo("Executing turn_left_90 command")
            self.publish_twist(twist)
        elif cmd == "rotate":
            twist.linear.x = 0.0
            twist.angular.z = 0.5   # Adjust as needed for rotation.
            self.current_turn_threshold = math.pi  # 180° turn.
            self.turning = True
            self.initial_theta = None
            rospy.loginfo("Executing rotate command")
            self.publish_twist(twist)
        else:
            rospy.logwarn("Unknown movement command received: %s", cmd)

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

    def publish_twist(self, twist):
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
        payload = cbor2.dumps(twist_dict)
        self.mqtt_handler.publish(MQTT_OPTIMISED_COMMAND, payload)
