# control/time_sync.py
#
# Time sync stub.
# Later: implement SNTP (WiFi), host-set time (CAN/USB), and/or RTC set.
#
# For now:
# - "is_time_valid()" checks if localtime year looks sane
# - "set_time_tuple()" allows host to set time (year,month,day,hour,min,sec,weekday,yearday)
# - "now_string()" used by storage/session naming if you want

import time

def is_time_valid(min_year=2024):
    try:
        y = time.localtime()[0]
        return y >= int(min_year)
    except Exception:
        return False

def set_time_tuple(t):
    """
    t is a tuple compatible with time.localtime() format:
      (year, month, mday, hour, minute, second, weekday, yearday)
    Note: On many ports, time/timezone/RTC setters differ.
    We'll later wire this to machine.RTC() if available.
    """
    try:
        from machine import RTC
        rtc = RTC()
        # RTC.datetime expects: (year, month, day, weekday, hour, minute, second, subseconds)
        year, month, mday, hour, minute, second, weekday, yearday = t
        rtc.datetime((year, month, mday, weekday, hour, minute, second, 0))
        return True
    except Exception:
        return False

def now_string():
    """
    Returns YYYY-MM-DD-HH-MM-SS if time valid, else None.
    """
    if not is_time_valid():
        return None
    try:
        y, mo, d, hh, mm, ss, wd, yd = time.localtime()
        return "{:04d}-{:02d}-{:02d}-{:02d}-{:02d}-{:02d}".format(y, mo, d, hh, mm, ss)
    except Exception:
        return None
