# control/session_logger.py
#
# Session logging: open file when battery attaches, write during run,
# close and finalize when battery removed.

import storage

class SessionLogger:
    def __init__(self):
        self.f = None
        self.filename = None
        self.active = False

    def start(self):
        if self.active:
            return
        self.f = storage.open_session()
        try:
            self.filename = getattr(self.f, "name", None)
        except:
            self.filename = None
        self.active = True
        self._w("# session start\n")

    def stop(self):
        if not self.active:
            return
        self._w("# session end\n")
        try:
            self.f.close()
        except:
            pass
        self.f = None
        self.active = False

    def _w(self, s):
        if not self.active or self.f is None:
            return
        try:
            self.f.write(s)
            self.f.flush()
        except:
            # best effort; ignore write failures
            pass

    def log_kv(self, **kv):
        """
        Writes one line like:
          key=value key=value ...
        """
        if not self.active:
            return
        parts = []
        for k, v in kv.items():
            parts.append("{}={}".format(k, v))
        self._w(" ".join(parts) + "\n")

    def log_text(self, line):
        self._w(str(line).rstrip() + "\n")
