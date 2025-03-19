import paho.mqtt.client as mqtt
import time
import json
import statistics

# Global counters for throughput
total_bytes = 0
start_time = time.time()

def on_message(client, userdata, msg):
    print(f"Received message on {msg.topic}")
    global total_bytes, start_time

    # Add the payload size (in bytes) for any message received.
    total_bytes += len(msg.payload)

    # Attempt to compute latency if the message is in JSON format.
    try:
        data = json.loads(msg.payload.decode('utf-8'))
        sent_timestamp = data.get("timestamp")
        if sent_timestamp:
            current_time_us = int(time.time() * 1e6)
            latency = current_time_us - sent_timestamp
            print(f"Message latency ({msg.topic}): {latency} us")
    except json.JSONDecodeError:
        # If the payload isn't JSON, simply print the raw message (optional)
        print(f"Non-JSON message on {msg.topic}: {msg.payload.decode('utf-8')}")
    
    # Every 30 seconds, calculate and print throughput.
    if time.time() - start_time >= 30:
        throughput = total_bytes / 30  # Bytes per second
        print(f"Throughput: {throughput:.2f} B/s over the last 30 seconds")
        # Reset counters for the next interval.
        total_bytes = 0
        start_time = time.time()

client = mqtt.Client()
client.on_message = on_message

# Replace with your MQTT broker's IP address.
client.connect("10.42.0.1", 1883, 60)
client.subscribe("epuck/#")
client.loop_forever()
