# config.py

# MQTT settings
MQTT_BROKER = "localhost"
MQTT_PORT = 1883

# MQTT topic names
MQTT_BATTERY = "robot/battery"
MQTT_ODOM = "robot/odom"
MQTT_SCAN = "robot/scan"
MQTT_IMU = "robot/imu"
MQTT_LATENCY = "robot/latency"
MQTT_THROUGHPUT = "robot/throughput"
MQTT_DATA = "robot/data"
MQTT_OPTIMISED_COMMAND = "optimised/command"
MQTT_RAW_COMMAND = "raw/command"

# ROS topic names
ROS_BATTERY = "/battery"
ROS_ODOM = "/odom"
ROS_SCAN = "/scan"
ROS_IMU = "/imu"
ROS_CMD_VEL = "/mobile_base/cmd_vel"
