# timekeeper.py

import time

def set_time_from_network(epoch):
    tm = time.localtime(epoch)
    time.settime(tm)

def get_timestamp():
    try:
        tm = time.localtime()
        if tm[0] < 2023:
            return None
        return "%04d-%02d-%02d_%02d-%02d-%02d" % tm[:6]
    except:
        return None
