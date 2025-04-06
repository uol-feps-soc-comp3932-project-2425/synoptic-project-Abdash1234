#!/usr/bin/env python3
import json
import csv

def parse_multiple_json(file_path):
    """
    Reads a file containing multiple JSON objects (possibly not separated by newlines)
    and returns a list of Python dictionaries.
    """
    decoder = json.JSONDecoder()
    objects = []
    with open(file_path, 'r') as f:
        content = f.read().strip()
    pos = 0
    while pos < len(content):
        try:
            obj, index = decoder.raw_decode(content, pos)
            objects.append(obj)
            pos = index
            # Skip any whitespace or newlines between JSON objects.
            while pos < len(content) and content[pos].isspace():
                pos += 1
        except json.JSONDecodeError as e:
            print(f"Error parsing at pos {pos}: {e}")
            break
    return objects

def flatten_dict(d, parent_key='', sep='_'):
    """
    Flattens a nested dictionary. For example:
      {'throughput': {'battery_msg_per_sec': 244.0}, 'latency': {'battery_latency_mean': 0.0018}}
    becomes:
      {'throughput_battery_msg_per_sec': 244.0, 'latency_battery_latency_mean': 0.0018}
    """
    items = {}
    for k, v in d.items():
        new_key = f"{parent_key}{sep}{k}" if parent_key else k
        if isinstance(v, dict):
            items.update(flatten_dict(v, new_key, sep=sep))
        else:
            items[new_key] = v
    return items

def write_csv_from_json_objects(json_objects, output_file):
    # Flatten all objects
    flattened = [flatten_dict(obj) for obj in json_objects]
    # Determine all unique keys (columns)
    fieldnames = sorted({key for obj in flattened for key in obj.keys()})
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
        writer.writeheader()
        for row in flattened:
            writer.writerow(row)

if __name__ == '__main__':
    input_file = "metrics.txt"    # Change to the path of your text file
    output_file = "metrics.csv"   # Output CSV filename
    json_objects = parse_multiple_json(input_file)
    if json_objects:
        write_csv_from_json_objects(json_objects, output_file)
        print("CSV conversion complete.")
    else:
        print("No valid JSON objects found.")
