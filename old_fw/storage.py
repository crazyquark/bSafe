# storage.py

import os
from timekeeper import get_timestamp

SESSIONS_DIR = "sessions"

def ensure_dir():
    if SESSIONS_DIR not in os.listdir():
        os.mkdir(SESSIONS_DIR)

def new_session_filename():
    ts = get_timestamp()
    if ts:
        return "{}/{}.txt".format(SESSIONS_DIR, ts.replace(":", "-"))
    return "{}/battery_{}.txt".format(SESSIONS_DIR, len(os.listdir(SESSIONS_DIR)))

def open_session():
    ensure_dir()
    fname = new_session_filename()
    return open(fname, "w")
