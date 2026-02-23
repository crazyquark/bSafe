# control/events.py
#
# Tiny ring-buffer event queue for cooperative main loop (or asyncio).
# Use for button presses, alerts, charger state transitions, comms commands, etc.

class EventQueue:
    def __init__(self, capacity=32):
        self._cap = int(capacity)
        self._buf = [None] * self._cap
        self._head = 0
        self._tail = 0
        self._count = 0

    def put(self, evt):
        """
        evt can be any object, but prefer tuples:
          ("btn", "LEFT")
          ("ina_alert", 1)
          ("chg_int", True)
        Returns True if queued, False if dropped.
        """
        if self._count >= self._cap:
            return False
        self._buf[self._tail] = evt
        self._tail = (self._tail + 1) % self._cap
        self._count += 1
        return True

    def get(self):
        if self._count == 0:
            return None
        evt = self._buf[self._head]
        self._buf[self._head] = None
        self._head = (self._head + 1) % self._cap
        self._count -= 1
        return evt

    def empty(self):
        return self._count == 0

    def size(self):
        return self._count

    def clear(self):
        self._head = self._tail = self._count = 0
        for i in range(self._cap):
            self._buf[i] = None
