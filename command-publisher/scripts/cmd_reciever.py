#!/usr/bin/env python3
import rospy
import cbor2
import paho.mqtt.client as mqtt
from geometry_msgs.msg import Twist

# MQTT settings
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC = "optimised/command"  # Topic for optimized command messages (in CBOR format)

# Global ROS publisher for Twist messages
cmd_pub = None

def on_mqtt_message(client, userdata, msg):
    global cmd_pub
    try:
        # Decode the CBOR payload into a dictionary
        command_data = cbor2.loads(msg.payload)
        rospy.loginfo("Received CBOR command: %s", command_data)
        
        # Create a Twist message from the decoded data
        twist = Twist()
        # Assuming command_data has a structure similar to:
        # {"linear": {"x": value, "y": value, "z": value}, "angular": {"x": value, "y": value, "z": value}}
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
            rospy.loginfo("Published Twist message to /mobile_base/cmd_vel: %s", twist)
        else:
            rospy.logwarn("ROS command publisher not initialized!")
    except Exception as e:
        rospy.logerr("Error processing CBOR command: %s", e)

def mqtt_to_ros_node():
    global cmd_pub
    rospy.init_node('mqtt_to_ros_command_bridge', anonymous=True)
    
    # Create a ROS publisher for the command topic
    cmd_pub = rospy.Publisher('/mobile_base/cmd_vel', Twist, queue_size=10)
    
    # Set up the MQTT client
    client = mqtt.Client()
    client.on_message = on_mqtt_message
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    
    # Subscribe to the MQTT topic with optimized command messages
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
