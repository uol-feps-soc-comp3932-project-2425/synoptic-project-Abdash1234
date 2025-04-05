#!/usr/bin/env python3
import tkinter as tk
import json
import time
import paho.mqtt.client as mqtt
import threading

# MQTT connection settings
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_CMD_TOPIC = "raw/command"      # Topic for sending raw commands
MQTT_ACK_TOPIC = "robot/command_ack"  # Topic for receiving command acknowledgments

# Global speed variable (adjust as needed)
speed = 2

# Dictionary to store pending commands with their timestamp for latency calculation
pending_commands = {}

def on_connect(client, userdata, flags, rc):
    print("Connected with result code", rc)
    # Subscribe to the acknowledgment topic once connected
    client.subscribe(MQTT_ACK_TOPIC)

def on_command_ack(client, userdata, msg):
    """
    Callback for processing command acknowledgment messages.
    Expects an acknowledgment message containing:
      - "command_id": the unique identifier of the original command.
      - "ack_timestamp": when the command was acknowledged.
    """
    try:
        ack_data = json.loads(msg.payload.decode())
        command_id = ack_data.get("command_id")
        ack_timestamp = ack_data.get("ack_timestamp")
        if command_id in pending_commands:
            original_timestamp = pending_commands.pop(command_id)
            latency = ack_timestamp - original_timestamp
            print(f"End-to-End Command Latency for command {command_id}: {latency:.3f} seconds")
        else:
            print("Received ack for unknown command id:", command_id)
    except Exception as e:
        print("Error processing acknowledgment:", e)

# Create a persistent MQTT client for both publishing and subscribing
mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect
mqtt_client.on_message = on_command_ack

def mqtt_loop():
    mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
    mqtt_client.loop_forever()

# Start the MQTT client loop in a separate thread
mqtt_thread = threading.Thread(target=mqtt_loop)
mqtt_thread.daemon = True
mqtt_thread.start()

def publish_command(command_data):
    """
    Publishes the given command_data (a dictionary) to the MQTT command topic.
    """
    payload = json.dumps(command_data)
    mqtt_client.publish(MQTT_CMD_TOPIC, payload)
    print("Published command:", command_data)

def send_command(command, extra_fields=None):
    """
    Helper function to send a command with a timestamp and unique command_id.
    Optionally update the message with extra fields (e.g., speed).
    """
    current_time = time.time()
    # Use the current time as a unique command ID (alternatively, use a counter or UUID)
    command_id = current_time
    data = {
        "command": command,
        "timestamp": current_time,
        "command_id": command_id
    }
    if extra_fields:
        data.update(extra_fields)
    # Record the time when the command is sent for latency calculation
    pending_commands[command_id] = current_time
    publish_command(data)

def send_go_forward():
    global speed
    send_command("go_forward", {"speed": speed})

def send_go_backwards():
    global speed
    send_command("go_backwards", {"speed": speed})

def send_stop():
    send_command("stop")

def send_turn_right():
    send_command("turn_right_90")

def send_turn_left():
    send_command("turn_left_90")

def send_rotate():
    send_command("rotate")

def increase_speed():
    global speed
    speed += 1
    speed_label.config(text=f"Speed: {speed}")

def decrease_speed():
    global speed
    if speed > 0:
        speed -= 1
    speed_label.config(text=f"Speed: {speed}")

def main():
    global speed_label, speed
    root = tk.Tk()
    root.title("MQTT Command Publisher")

    # Frame for speed control
    speed_frame = tk.Frame(root)
    speed_frame.pack(pady=5)
    
    speed_label = tk.Label(speed_frame, text=f"Speed: {speed}", font=("Arial", 12))
    speed_label.pack(side=tk.LEFT, padx=5)
    
    minus_button = tk.Button(speed_frame, text="-", command=decrease_speed, width=3)
    minus_button.pack(side=tk.LEFT, padx=5)
    
    plus_button = tk.Button(speed_frame, text="+", command=increase_speed, width=3)
    plus_button.pack(side=tk.LEFT, padx=5)

    # Create buttons for each command
    tk.Button(root, text="Go Forward", command=send_go_forward, width=20).pack(pady=5)
    tk.Button(root, text="Go Backwards", command=send_go_backwards, width=20).pack(pady=5)
    tk.Button(root, text="Stop", command=send_stop, width=20).pack(pady=5)
    tk.Button(root, text="Turn Right 90°", command=send_turn_right, width=20).pack(pady=5)
    tk.Button(root, text="Turn Left 90°", command=send_turn_left, width=20).pack(pady=5)
    tk.Button(root, text="Rotate 180°", command=send_rotate, width=20).pack(pady=5)

    root.mainloop()

if __name__ == '__main__':
    main()
