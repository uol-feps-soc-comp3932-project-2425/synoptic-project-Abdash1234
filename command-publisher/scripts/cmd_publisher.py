#!/usr/bin/env python3
import tkinter as tk
import json
import paho.mqtt.client as mqtt

# MQTT connection settings
MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_TOPIC = "raw/command"  # Topic for sending raw commands

# Global speed variable (adjust as needed)
speed = 2

def publish_command(command_data):
    """
    Publishes the given command_data (a dictionary) to the MQTT topic.
    """
    client = mqtt.Client()
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.loop_start()
    payload = json.dumps(command_data)
    client.publish(MQTT_TOPIC, payload)
    print("Published command:", command_data)
    client.loop_stop()
    client.disconnect()

def send_go_forward():
    global speed
    command = {"command": "go_forward", "speed": speed}
    publish_command(command)

def send_go_backwards():
    global speed
    command = {"command": "go_backwards", "speed": speed}
    publish_command(command)

def send_stop():
    command = {"command": "stop"}
    publish_command(command)

def send_turn_right():
    command = {"command": "turn_right_90"}
    publish_command(command)

def send_turn_left():
    command = {"command": "turn_left_90"}
    publish_command(command)

def send_rotate():
    command = {"command": "rotate"}
    publish_command(command)

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
