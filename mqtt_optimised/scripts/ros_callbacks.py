# ros_callbacks.py

import rospy
import cbor2
import config  # Contains MQTT_BATTERY, etc.
from sensor_msgs.msg import BatteryState, LaserScan, Imu
from nav_msgs.msg import Odometry

def battery_callback(batt_msg, mqtt_handler, metrics):
    """
    Callback for processing BatteryState messages.
    
    This function:
      - Computes the latency of the message.
      - Logs a warning if latency exceeds a threshold.
      - Packages battery data into a dictionary.
      - Serializes the data using CBOR.
      - Publishes the serialized payload via the MQTT handler.
      - Logs processing time.
    """
    # Record start time for processing
    start_time = rospy.Time.now().to_sec()
    metrics.update_throughput("battery", 1)

    # Compute latency: difference between current time and message timestamp.
    current_time = rospy.Time.now().to_sec()
    original_stamp = batt_msg.header.stamp.to_sec()
    latency = current_time - original_stamp
    metrics.record_latency("battery", latency)
    # rospy.loginfo("Battery callback latency: %.3f seconds", latency)
    
    # If latency is high, log a warning (or trigger an event if you have an event system)
    latency_threshold = 0.1
    if latency > latency_threshold:
        rospy.logwarn("High latency in battery message: %.3f seconds", latency)
        # You might trigger additional actions here (e.g., adjusting QoS)

    # Prepare the battery data payload as a dictionary
    battery_data = {
        "header": {
            "frame_id": batt_msg.header.frame_id,
            "stamp": {
                "secs": batt_msg.header.stamp.secs,
                "nsecs": batt_msg.header.stamp.nsecs
            }
        },
        "voltage": batt_msg.voltage,
        "percentage": batt_msg.percentage,
        "present": batt_msg.present
    }

    # Serialize the battery data using CBOR
    payload = cbor2.dumps(battery_data)

    # Publish the serialized payload to the MQTT battery topic
    mqtt_handler.publish(config.MQTT_BATTERY, payload)
    # rospy.loginfo("Published battery data to MQTT (result: %s)", result)

    # Record finish time and compute processing time
    finish_time = rospy.Time.now().to_sec()
    processing_time = finish_time - start_time
    # rospy.loginfo("Battery callback processing time: %.3f seconds", processing_time)
    metrics.processing_records["battery"].append(processing_time)
    metrics.update_bandwidth("battery", len(config.MQTT_ODOM.encode('utf-8')) + len(payload) + 4)

def odom_callback(odom_msg, mqtt_handler, metrics):
    """
    Callback for processing Odometry messages.
    
    This function:
      - Computes the latency of the message.
      - Logs the latency.
      - Packages odometry data (header, position, orientation, velocities) into a dictionary.
      - Serializes the data using CBOR.
      - Publishes the serialized payload to the MQTT odometry topic.
      - Logs the processing time.
    """
    # Record the start time for processing
    start_time = rospy.Time.now().to_sec()
    metrics.update_throughput("odom", 1)
    
    # Compute latency: current time minus the message's timestamp
    current_time = rospy.Time.now().to_sec()
    original_stamp = odom_msg.header.stamp.to_sec()
    latency = current_time - original_stamp
    metrics.record_latency("odom", latency)
    # rospy.loginfo("Odom callback latency: %.3f seconds", latency)
    
    # Prepare the odometry data payload
    odom_data = {
        "header": {
            "frame_id": odom_msg.header.frame_id,
            "stamp": {
                "secs": odom_msg.header.stamp.secs,
                "nsecs": odom_msg.header.stamp.nsecs
            }
        },
        "position": {
            "x": odom_msg.pose.pose.position.x,
            "y": odom_msg.pose.pose.position.y,
            "z": odom_msg.pose.pose.position.z
        },
        "orientation": {
            "x": odom_msg.pose.pose.orientation.x,
            "y": odom_msg.pose.pose.orientation.y,
            "z": odom_msg.pose.pose.orientation.z,
            "w": odom_msg.pose.pose.orientation.w
        },
        "linear_velocity": odom_msg.twist.twist.linear.x,
        "angular_velocity": odom_msg.twist.twist.angular.z
    }
    
    # Serialize the dictionary using CBOR
    payload = cbor2.dumps(odom_data)
    
    # Publish the serialized data to the MQTT odometry topic
    mqtt_handler.publish(config.MQTT_ODOM, payload)
    # rospy.loginfo("Published odometry data to MQTT (result: %s)", result)
    
    # Record finish time and compute total processing time
    finish_time = rospy.Time.now().to_sec()
    processing_time = finish_time - start_time
    # rospy.loginfo("Odom callback processing time: %.3f seconds", processing_time)
    metrics.record_processing_time("odom", processing_time)
    metrics.update_bandwidth("odom", len(config.MQTT_ODOM.encode('utf-8')) + len(payload) + 4)

def scan_callback(scan_msg, mqtt_handler, metrics):
    """
    Process a LaserScan message:
      - Update throughput and record latency.
      - Package scan data and publish via MQTT.
      - Record processing time.
    """
    start_time = rospy.Time.now().to_sec()
    metrics.update_throughput("scan", 1)

    current_time = rospy.Time.now().to_sec()
    latency = current_time - scan_msg.header.stamp.to_sec()
    metrics.record_latency("scan", latency)
    # rospy.loginfo("Scan callback latency: %.3f seconds", latency)

    scan_data = {
        "header": {
            "frame_id": scan_msg.header.frame_id,
            "stamp": {
                "secs": scan_msg.header.stamp.secs,
                "nsecs": scan_msg.header.stamp.nsecs
            }
        },
        "angle_min": scan_msg.angle_min,
        "angle_max": scan_msg.angle_max,
        "angle_increment": scan_msg.angle_increment,
        "ranges": list(scan_msg.ranges)
    }
    payload = cbor2.dumps(scan_data)
    mqtt_handler.publish(config.MQTT_SCAN, payload)
    # rospy.loginfo("Published scan data to MQTT (result: %s)", result)

    finish_time = rospy.Time.now().to_sec()
    processing_time = finish_time - start_time
    # rospy.loginfo("Scan callback processing time: %.3f seconds", processing_time)
    metrics.record_processing_time("scan", processing_time)
    metrics.update_bandwidth("scan", len(config.MQTT_SCAN.encode('utf-8')) + len(payload) + 4)

def imu_callback(imu_msg, mqtt_handler, metrics):
    """
    Process an IMU message:
      - Update throughput and record latency.
      - Package IMU data and publish via MQTT.
      - Record processing time.
    """
    start_time = rospy.Time.now().to_sec()
    metrics.update_throughput("imu", 1)

    current_time = rospy.Time.now().to_sec()
    latency = current_time - imu_msg.header.stamp.to_sec()
    metrics.record_latency("imu", latency)
    # rospy.loginfo("IMU callback latency: %.3f seconds", latency)

    imu_data = {
        "header": {
            "frame_id": imu_msg.header.frame_id,
            "stamp": {
                "secs": imu_msg.header.stamp.secs,
                "nsecs": imu_msg.header.stamp.nsecs
            }
        },
        "linear_acceleration": {
            "x": imu_msg.linear_acceleration.x,
            "y": imu_msg.linear_acceleration.y,
            "z": imu_msg.linear_acceleration.z
        },
        "angular_velocity": {
            "x": imu_msg.angular_velocity.x,
            "y": imu_msg.angular_velocity.y,
            "z": imu_msg.angular_velocity.z
        },
        "orientation": {
            "x": imu_msg.orientation.x,
            "y": imu_msg.orientation.y,
            "z": imu_msg.orientation.z,
            "w": imu_msg.orientation.w
        }
    }
    payload = cbor2.dumps(imu_data)
    mqtt_handler.publish(config.MQTT_IMU, payload)
    # rospy.loginfo("Published IMU data to MQTT (result: %s)", result)

    finish_time = rospy.Time.now().to_sec()
    processing_time = finish_time - start_time
    # rospy.loginfo("IMU callback processing time: %.3f seconds", processing_time)
    metrics.record_processing_time("imu", processing_time)
    metrics.update_bandwidth("imu", len(config.MQTT_IMU.encode('utf-8')) + len(payload) + 4)
