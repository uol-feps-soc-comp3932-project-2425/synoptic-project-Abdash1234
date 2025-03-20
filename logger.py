import paho.mqtt.client as mqtt
import time
import json
import statistics

# Global counters for throughput and latency measurements.
total_bytes = 0
latency_list = []
start_time = time.time()
lost_messages = 0
received_messages = 0
expected_seq = None

# Globals for relative time calibration.
base_monotonic = None
base_sent_timestamp = None

def log_to_file(data):
    # Append the data to the log file.
    with open("mqtt_log.txt", "a") as log_file:
        log_file.write(data + "\n")

def on_message(client, userdata, msg):
    global total_bytes, start_time, latency_list, lost_messages, received_messages, expected_seq
    global base_monotonic, base_sent_timestamp

    # Log the raw message.
    raw_msg = f"Topic: {msg.topic}, Data: {msg.payload.decode('utf-8')}"
    print(raw_msg)
    log_to_file(raw_msg)

    # Update counters.
    total_bytes += len(msg.payload)
    received_messages += 1

    # Attempt to process the message as JSON.
    try:
        data = json.loads(msg.payload.decode('utf-8'))
        seq = data.get("seq")
        sent_timestamp = data.get("timestamp")
        if sent_timestamp:
            # Sequence tracking for message loss.
            if expected_seq is None:
                expected_seq = seq + 1
            else:
                if seq != expected_seq:
                    lost = seq - expected_seq
                    lost_messages += lost
                    expected_seq = seq + 1
                else:
                    expected_seq += 1
            
            # Use monotonic time for relative latency measurement.
            current_monotonic = int(time.monotonic() * 1e6)
            if base_monotonic is None:
                # Calibrate with the first message received.
                base_monotonic = current_monotonic
                base_sent_timestamp = sent_timestamp

            elapsed_receiver = current_monotonic - base_monotonic
            elapsed_sender = sent_timestamp - base_sent_timestamp
            latency = elapsed_receiver - elapsed_sender

            latency_msg = f"Message latency ({msg.topic}): {msg.payload.decode('utf-8')} : {latency} us"
            print(latency_msg)
            log_to_file(latency_msg)
            latency_list.append(latency)
        else:
            no_timestamp_msg = f"Non-JSON or missing fields on {msg.topic}: {msg.payload.decode('utf-8')}"
            print(no_timestamp_msg)
            log_to_file(no_timestamp_msg)
    except json.JSONDecodeError:
        non_json_msg = f"Non-JSON message on {msg.topic}: {msg.payload.decode('utf-8')}"
        print(non_json_msg)
        log_to_file(non_json_msg)
    
    # Every 60 seconds, calculate and log throughput, jitter, and delivery success rate.
    if time.time() - start_time >= 60:
        throughput = total_bytes / 60  # Bytes per second.
        jitter = statistics.stdev(latency_list) if len(latency_list) > 1 else 0
        success_rate = (received_messages / (received_messages + lost_messages)) * 100 if (received_messages + lost_messages) > 0 else 100
        
        metrics_output = (
            "\n--- Metrics for the last 60 seconds ---\n"
            f"Throughput: {throughput:.2f} B/s\n"
            f"Jitter: {jitter:.2f} us\n"
            f"Messages Received: {received_messages}\n"
            f"Lost Messages: {lost_messages}\n"
            f"Message Delivery Success Rate: {success_rate:.2f}%\n"
        )
        
        print(metrics_output)
        log_to_file(metrics_output)
        
        # Reset counters for the next interval.
        total_bytes = 0
        latency_list = []
        start_time = time.time()
        lost_messages = 0
        received_messages = 0
        expected_seq = None
        # Reset the base calibration for the next interval.
        base_monotonic = None
        base_sent_timestamp = None

client = mqtt.Client()
client.on_message = on_message

# Replace with your MQTT broker's IP address.
client.connect("10.42.0.1", 1883, 60)
client.subscribe("epuck/#")
client.loop_forever()
