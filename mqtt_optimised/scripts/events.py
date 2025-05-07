# events.py
LOW_BATTERY_EVENT = "low_battery"
HIGH_LATENCY_EVENT = "high_latency"
HIGH_ERROR_RATE_EVENT = "high_error_rate"
NETWORK_CONGESTION_EVENT = "network_congestion"
DUPLICATE_MESSAGE_EVENT = "duplicate_message"

# fleshed out events
def on_low_battery(battery_level, qos_manager): #battery level dropped below a threshold
    print("Low battery event triggered! Battery level: %.2f", battery_level)
    qos_manager.increase_qos()

def on_high_latency(topic, latency,qos_manager): #sharp spike in latency
    qos_manager.increase_qos()

def high_error_rate(topic,error_rate, qos_manager): #sharp spike in error rate
    qos_manager.set_qos((2))


"""Potential New Events - need to be fleshed out """
# def obstacle_detected(): # obstacle has been detected by the sensors - obstacle avoidance would take place following this
#     print("Obstacle Detected")

# def lost_connection():
#     print("broker connection lost")

# def broker_connect():
#     print("robot connected to broker")

# def idle_timeout():
#     print("stopping sensor readings due to no movement")

# def movement_resumed():
#     print("command recieved, movement resumed")

# def on_network_congestion(bandwidth_usage):
#     print("Network congestion detected! Bandwidth usage: %d bytes/s", bandwidth_usage)
#     # Example: Increase aggregator flush interval or reduce message frequency.

# def on_duplicate_message(duplicate_count, duplicate_rate):
#     print("Duplicate messages detected: %d duplicates (%.2f%%)", duplicate_count, duplicate_rate)
#     # Example: Log diagnostics or adjust the sending frequency.