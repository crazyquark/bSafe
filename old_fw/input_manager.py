# control/input_manager.py
#
# Debounced input event generator.
# Currently consumes Board.poll_button_edges() which already emits edge presses.
# Provides a consistent interface for main loop.

import time

class InputManager:
    def __init__(self, board, event_queue, debounce_ms=120):
        self.board = board
        self.q = event_queue
        self.debounce_ms = int(debounce_ms)

        # last accepted press time per key
        self._last_ms = {
            "menu": 0,
            "left": 0,
            "right": 0,
            "ok": 0,
        }

    def _accept(self, key):
        now = time.ticks_ms()
        last = self._last_ms.get(key, 0)
        if time.ticks_diff(now, last) < self.debounce_ms:
            return False
        self._last_ms[key] = now
        return True

    def poll(self):
        """
        Call frequently (e.g. every 20–50ms).
        Pushes events to queue:
          ("btn", "MENU"), ("btn","LEFT"), ("btn","RIGHT"), ("btn","OK")
        """
        ev = self.board.poll_button_edges()

        if ev.get("menu") and self._accept("menu"):
            self.q.put(("btn", "MENU"))

        if ev.get("left") and self._accept("left"):
            self.q.put(("btn", "LEFT"))

        if ev.get("right") and self._accept("right"):
            self.q.put(("btn", "RIGHT"))

        if ev.get("ok") and self._accept("ok"):
            self.q.put(("btn", "OK"))
