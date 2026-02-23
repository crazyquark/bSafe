# settings.py

import json
import os

SETTINGS_FILE = "settings.json"

DEFAULT_SETTINGS = {
    "chemistry": "LFP",
    "voltage_offset_mv": 0,        # -200 … +50
    "cell_capacity_mah": 3000,
    "parallel_cells": 1,

    "charge_c_rate": 0.5,
    "discharge_c_rate": 2.0,

    "wait_mode": "NO_WAIT",         # NO_WAIT, 4H, 8H, 12H, 16H
    "discharge_target": "-100%",    # KEEP_FULL … -100%

    "mode_can": True,
    "mode_wifi": False,
    "mode_usb": True,
    "lock_settings_on_net": False,
}

def load():
    if SETTINGS_FILE in os.listdir():
        with open(SETTINGS_FILE, "r") as f:
            return json.load(f)
    save(DEFAULT_SETTINGS)
    return DEFAULT_SETTINGS.copy()

def save(settings):
    with open(SETTINGS_FILE, "w") as f:
        json.dump(settings, f)

settings = load()
