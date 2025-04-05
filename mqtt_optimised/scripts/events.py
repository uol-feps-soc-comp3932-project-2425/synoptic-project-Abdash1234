# events.py
LOW_BATTERY_EVENT = "low_battery"
HIGH_LATENCY_EVENT = "high_latency"
HIGH_ERROR_RATE_EVENT = "high_error_rate"
NETWORK_CONGESTION_EVENT = "network_congestion"
DUPLICATE_MESSAGE_EVENT = "duplicate_message"

def on_low_battery(battery_level):
    # Do something, like force an aggregator flush or adjust QoS
    print("Low battery event triggered! Battery level: %.2f", battery_level)

def on_high_latency(topic, latency,qos_manager):
    # print("High latency detected on %s: %.3f sec", topic, latency)
    qos_manager.increase_qos()

    # if topic == "/battery":
    #     print("flush buffer")
    # # Possibly adjust QoS here...

# def on_high_error_rate(topic, error_rate):
#     print("High error rate on %s: %.2f%%", topic, error_rate)
#     # Example: Reconnect MQTT or trigger diagnostics.

# def on_network_congestion(bandwidth_usage):
#     print("Network congestion detected! Bandwidth usage: %d bytes/s", bandwidth_usage)
#     # Example: Increase aggregator flush interval or reduce message frequency.

# def on_duplicate_message(duplicate_count, duplicate_rate):
#     print("Duplicate messages detected: %d duplicates (%.2f%%)", duplicate_count, duplicate_rate)
#     # Example: Log diagnostics or adjust the sending frequency.