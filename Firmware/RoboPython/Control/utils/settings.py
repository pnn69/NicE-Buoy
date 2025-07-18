import json
import os
import tempfile
from robobuoy_pid_ui.constants import SETTINGS_FILE

def save_values(data):
    try:
        with tempfile.NamedTemporaryFile('w', delete=False, dir='.') as tf:
            json.dump(data, tf, indent=4)
            temp_name = tf.name
        os.replace(temp_name, SETTINGS_FILE)
    except Exception as e:
        print("Error saving values:", e)


def load_values():
    if not os.path.isfile(SETTINGS_FILE):
        return {}
    try:
        with open(SETTINGS_FILE, 'r') as f:
            return json.load(f)
    except Exception as e:
        print("Error loading saved values:", e)
        return {}
