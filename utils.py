import os
import json
from datetime import datetime

def make_directory(path: str):
    os.makedirs(path, exist_ok=True)

def link_prev_next(sample_data_list):
    from collections import defaultdict
    by_sensor = defaultdict(list)
    for rec in sample_data_list:
        by_sensor[rec["sensor_token"]].append(rec)
    for records in by_sensor.values():
        records.sort(key=lambda r: r["timestamp"])
        for i, r in enumerate(records):
            r["prev"] = records[i-1]["token"] if i > 0 else ""
            r["next"] = records[i+1]["token"] if i < len(records)-1 else ""

def save_json(path: str, obj):
    with open(path, "w") as f:
        json.dump(obj, f, indent=2)
