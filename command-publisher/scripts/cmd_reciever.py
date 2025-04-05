#!/usr/bin/env python3
import rospy
import cbor2
import time
import json
import paho.mqtt.client as mqtt
from geometry_msgs.msg import Twist

# MQTT settings
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC = "optimised/command"      # Topic for optimized command messages (in CBOR or JSON format)
MQTT_ACK_TOPIC = "robot/command_ack"  # Topic for publishing acknowledgments with metrics

# Global ROS publisher for Twist messages
cmd_pub = None

# Global variable to keep track of the previous latency (in seconds) for jitter calculation
previous_latency = None

# Global variables for message loss calculation using the sequence number
last_seq = None       # Last sequence number received
total_lost = 0        # Total number of lost messages (cumulative)
total_received = 0    # Total number of commands received

def on_mqtt_message(client, userdata, msg):
    global cmd_pub, previous_latency, last_seq, total_lost, total_received
    processing_start = time.time()  # Start processing timer
    try:
        # Attempt to decode the incoming payload using CBOR.
        # If it fails, fall back to JSON.
        try:
            command_data = cbor2.loads(msg.payload)
        except Exception as e:
            rospy.logwarn("CBOR decoding failed, trying JSON: %s", e)
            command_data = json.loads(msg.payload.decode('utf-8'))
        
        rospy.loginfo("Received command: %s", command_data)
        
        # Extract key fields: send timestamp, command_id, and sequence number
        send_timestamp = command_data.get("timestamp")
        command_id = command_data.get("command_id", "N/A")
        seq = command_data.get("seq", None)
        ack_timestamp = time.time()
        
        # Message loss calculation using sequence numbers:
        lost_current = 0
        if seq is not None:
            if last_seq is None:
                last_seq = seq
            else:
                if seq > last_seq:
                    lost_current = seq - last_seq - 1
                    total_lost += lost_current
                    last_seq = seq
                else:
                    lost_current = 0
            total_received += 1
        else:
            rospy.logwarn("No sequence number found in command")
        
        # Compute latency if send_timestamp is available (in seconds)
        if send_timestamp:
            latency = ack_timestamp - send_timestamp
            rospy.loginfo("End-to-End Command Latency (Receiver): %.0f ms", latency * 1000)
        else:
            latency = -1
            rospy.logwarn("No timestamp in received command")
        
        # Compute jitter as the absolute difference between current and previous latency (in ms)
        if send_timestamp and previous_latency is not None and latency >= 0:
            jitter_ms = abs(latency - previous_latency) * 1000.0
        else:
            jitter_ms = 0.0
        previous_latency = latency
        
        # Determine the payload size in bytes
        payload_size = len(msg.payload)
        
        # Create a Twist message from the decoded command data
        twist = Twist()
        if "linear" in command_data:
            twist.linear.x = command_data["linear"].get("x", 0.0)
            twist.linear.y = command_data["linear"].get("y", 0.0)
            twist.linear.z = command_data["linear"].get("z", 0.0)
        if "angular" in command_data:
            twist.angular.x = command_data["angular"].get("x", 0.0)
            twist.angular.y = command_data["angular"].get("y", 0.0)
            twist.angular.z = command_data["angular"].get("z", 0.0)
        
        # Publish the Twist message to the ROS topic
        if cmd_pub is not None:
            cmd_pub.publish(twist)
            rospy.loginfo("Published Twist message to /mobile_base/cmd_vel")
        else:
            rospy.logwarn("ROS command publisher not initialized!")
        
        processing_end = time.time()  # End processing timer
        processing_time_ms = (processing_end - processing_start) * 1000.0
        
        # Calculate cumulative loss percentage
        total_expected = total_received + total_lost  # Total messages that should have been seen so far
        loss_pct = (total_lost / total_expected) * 100 if total_expected > 0 else 0
        
        # Prepare an acknowledgment message with key metrics, including processing time
        ack_msg = {
            "command_id": command_id,
            "seq": seq,
            "send_timestamp": send_timestamp,
            "ack_timestamp": ack_timestamp,
            "latency_ms": latency * 1000 if latency >= 0 else None,
            "jitter_ms": jitter_ms,
            "payload_size": payload_size,
            "lost_since_last": lost_current,
            "total_lost": total_lost,
            "loss_pct": loss_pct,
            "processing_time_ms": processing_time_ms
        }
        
        # Convert the acknowledgment message to JSON and publish it on the ack topic
        ack_payload = json.dumps(ack_msg)
        client.publish(MQTT_ACK_TOPIC, ack_payload)
        rospy.loginfo("Published ack message: %s", ack_msg)
        
    except Exception as e:
        rospy.logerr("Error processing command: %s", e)

def mqtt_to_ros_node():
    global cmd_pub
    rospy.init_node('mqtt_to_ros_command_bridge', anonymous=True)
    
    # Create a ROS publisher for the command topic (/mobile_base/cmd_vel)
    cmd_pub = rospy.Publisher('/mobile_base/cmd_vel', Twist, queue_size=10)
    
    # Set up the MQTT client and assign the on_message callback
    client = mqtt.Client()
    client.on_message = on_mqtt_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    
    # Subscribe to the MQTT topic for optimized command messages
    client.subscribe(MQTT_TOPIC)
    
    # Start the MQTT network loop in a separate thread
    client.loop_start()
    
    rospy.loginfo("MQTT to ROS Command Bridge Node started, listening on MQTT topic: %s", MQTT_TOPIC)
    
    # Keep the ROS node running
    rospy.spin()
    
    # When ROS shuts down, stop the MQTT loop
    client.loop_stop()

if __name__ == '__main__':
    try:
        mqtt_to_ros_node()
    except rospy.ROSInterruptException:
        pass
