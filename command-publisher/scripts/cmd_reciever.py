#!/usr/bin/env python3
import rospy
import cbor2
import time
import json
import paho.mqtt.client as mqtt
from geometry_msgs.msg import Twist
import csv
import os

# MQTT settings
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC = "optimised/command"      # Topic for optimized command messages (in CBOR or JSON format)
MQTT_ACK_TOPIC = "robot/command_ack"  # Topic for publishing acknowledgments with metrics
CSV_FILE = "/home/abdullah/catkin_ws/src/synoptic-project-Abdash1234/csv_files/receiver.csv"

# Global ROS publisher for Twist messages
cmd_pub = None

# Global variable to keep track of the previous latency (in seconds) for jitter calculation
previous_latency = None

# Global variables for message loss calculation using the sequence number
last_seq = None       # Last sequence number received
total_lost = 0        # Total number of lost messages (cumulative)
total_received = 0    # Total number of commands received

def log_ack_to_csv(ack_msg, csv_file=CSV_FILE):
    """
    Logs the ack_msg dictionary to a CSV file.
    
    ack_msg: A dictionary containing the acknowledgment metrics.
    csv_file: Path to the CSV file.
    """
    # Define the CSV header
    header = ["timestamp", "seq", "latency_ms", "jitter_ms", "payload_size", 
              "lost_since_last", "total_lost", "loss_pct", "processing_time_ms"]
    
    # Check if file exists and is non-empty.
    file_exists = os.path.exists(csv_file) and os.path.getsize(csv_file) > 0
    
    # Get current time for logging purposes (optional)
    current_time = rospy.get_time()
    
    # Prepare the row data based on the ack_msg
    row = [
        current_time,
        ack_msg.get("seq"),
        ack_msg.get("latency_ms"),
        ack_msg.get("jitter_ms"),
        ack_msg.get("payload_size"),
        ack_msg.get("lost_since_last"),
        ack_msg.get("total_lost"),
        ack_msg.get("loss_pct"),
        ack_msg.get("processing_time_ms")
    ]
    
    try:
        with open(csv_file, mode="a", newline="") as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow(header)
            writer.writerow(row)
        rospy.loginfo("Logged ack metrics to CSV.")
    except Exception as e:
        rospy.logerr("Error writing to CSV: %s", e)

def on_mqtt_message(client, userdata, msg):
    global cmd_pub, previous_latency, last_seq, total_lost, total_received
    processing_start = time.time()  # Start processing timer
    try:
        # Attempt to decode the incoming payload using CBOR. If it fails, use JSON.
        try:
            command_data = cbor2.loads(msg.payload)
        except Exception as e:
            rospy.logwarn("CBOR decoding failed, trying JSON: %s", e)
            command_data = json.loads(msg.payload.decode('utf-8'))
        
        # rospy.loginfo("Received command: %s", command_data)
        
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
        # else:
        #     rospy.logwarn("No sequence number found in command")
        
        # Compute latency if send_timestamp is available (in seconds)
        if send_timestamp:
            latency = ack_timestamp - send_timestamp
            # rospy.loginfo("End-to-End Command Latency (Receiver): %.0f ms", latency * 1000)
        else:
            latency = -1
            # rospy.logwarn("No timestamp in received command")
        
        # Compute jitter as the absolute difference between current and previous latency (in ms)
        if send_timestamp and previous_latency is not None and latency >= 0:
            jitter_ms = abs(latency - previous_latency) * 1000.0
        else:
            jitter_ms = 0.0
        previous_latency = latency
        
        # Determine the payload size in bytes
        payload_size = len(msg.payload)
        
        # Create a Twist message from the decoded command data (if applicable)
        from geometry_msgs.msg import Twist  # Import here if not globally imported
        twist = Twist()
        if "linear" in command_data:
            twist.linear.x = command_data["linear"].get("x", 0.0)
            twist.linear.y = command_data["linear"].get("y", 0.0)
            twist.linear.z = command_data["linear"].get("z", 0.0)
        if "angular" in command_data:
            twist.angular.x = command_data["angular"].get("x", 0.0)
            twist.angular.y = command_data["angular"].get("y", 0.0)
            twist.angular.z = command_data["angular"].get("z", 0.0)
        
        # Publish the Twist message to the ROS topic if publisher is available
        if cmd_pub is not None:
            cmd_pub.publish(twist)
            # rospy.loginfo("Published Twist message to /mobile_base/cmd_vel")
        # else:
        #     rospy.logwarn("ROS command publisher not initialized!")
        
        processing_end = time.time()  # End processing timer
        processing_time_ms = (processing_end - processing_start) * 1000.0
        
        # Calculate cumulative loss percentage
        total_expected = total_received + total_lost  # Total messages expected so far
        loss_pct = (total_lost / total_expected) * 100 if total_expected > 0 else 0
        
        # Prepare an acknowledgment message with key metrics
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
        
        # Format the acknowledgment message for nicer output
        formatted_ack = (
            "\n----- Acknowledgment -----\n"
            # f"Command ID         : {ack_msg['command_id']}\n"
            f"Sequence           : {ack_msg['seq']}\n"
            # f"Send Timestamp     : {ack_msg['send_timestamp']}\n"
            # f"Ack Timestamp      : {ack_msg['ack_timestamp']}\n"
            f"Latency            : {ack_msg['latency_ms']:.3f} ms\n"
            f"Jitter             : {ack_msg['jitter_ms']:.3f} ms\n"
            f"Payload Size       : {ack_msg['payload_size']} bytes\n"
            f"Lost Packets (this): {ack_msg['lost_since_last']}\n"
            f"Total Lost         : {ack_msg['total_lost']}\n"
            f"Loss Percentage    : {ack_msg['loss_pct']:.2f}%\n"
            f"Processing Time    : {ack_msg['processing_time_ms']:.3f} ms\n"
            "---------------------------\n"
        )
        
        # Publish the acknowledgment message on the MQTT_ACK_TOPIC
        ack_payload = json.dumps(ack_msg)
        client.publish(MQTT_ACK_TOPIC, ack_payload)
        rospy.loginfo(formatted_ack)
        log_ack_to_csv(ack_msg)
        
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
    
    # rospy.loginfo("MQTT to ROS Command Bridge Node started, listening on MQTT topic: %s", MQTT_TOPIC)
    
    # Keep the ROS node running
    rospy.spin()
    
    # When ROS shuts down, stop the MQTT loop
    client.loop_stop()

if __name__ == '__main__':
    try:
        mqtt_to_ros_node()
    except rospy.ROSInterruptException:
        pass
