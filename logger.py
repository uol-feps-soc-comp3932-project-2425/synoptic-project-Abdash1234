import paho.mqtt.client as mqtt
import time
import json
import statistics

# Global counters for throughput and a list for latency measurements.
total_bytes = 0
latency_list = []
start_time = time.time()

def on_message(client, userdata, msg):
    global total_bytes, start_time, latency_list

    # Add the payload size (in bytes) for any message received.
    total_bytes += len(msg.payload)

    # Attempt to compute latency if the message is in JSON format.
    try:
        data = json.loads(msg.payload.decode('utf-8'))
        sent_timestamp = data.get("timestamp")
        if sent_timestamp:
            current_time_us = int(time.time() * 1e6)
            latency = current_time_us - sent_timestamp
            print(f"Message latency ({msg.topic}):  : ({msg.payload.decode('utf-8')}) : ({latency}) us")
            # Add latency to the list for jitter calculation.
            latency_list.append(latency)
    except json.JSONDecodeError:
        # If the payload isn't JSON, simply print the raw message.
        print(f"Non-JSON message on {msg.topic}: {msg.payload.decode('utf-8')}")
    
    # Every 30 seconds, calculate and print throughput and jitter.
    if time.time() - start_time >= 30:
        throughput = total_bytes / 30  # Bytes per second
        jitter = statistics.stdev(latency_list) if len(latency_list) > 1 else 0
        print(f"\n--- Metrics for the last 30 seconds ---")
        print(f"Throughput: {throughput:.2f} B/s")
        print(f"Jitter: {jitter:.2f} us")
        print(f"Total messages received: {len(latency_list)}\n")
        # Reset counters for the next interval.
        total_bytes = 0
        latency_list = []
        start_time = time.time()

client = mqtt.Client()
client.on_message = on_message

# Replace with your MQTT broker's IP address.
client.connect("10.42.0.1", 1883, 60)
client.subscribe("epuck/#")
client.loop_forever()
