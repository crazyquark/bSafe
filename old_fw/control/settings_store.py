# control/settings_store.py
#
# Settings persistence with safe-write.
# Uses JSON because it's easy to inspect and edit.
#
# Expected:
# - settings.py provides DEFAULT_SETTINGS dict
# - settings.json is runtime-updated

import ujson
import os

SETTINGS_PATH = "settings.json"
TMP_PATH = "settings.tmp"

class SettingsStore:
    def __init__(self, *, path=SETTINGS_PATH):
        self.path = path
        self.data = None

    def load(self, default_settings: dict):
        """
        Loads settings from JSON; if missing or invalid, writes defaults.
        Returns the dict used.
        """
        d = None
        try:
            with open(self.path, "r") as f:
                d = ujson.load(f)
        except Exception:
            d = None

        if not isinstance(d, dict):
            d = dict(default_settings)
            self.save(d)

        # Merge any new default keys (forward compatibility)
        changed = False
        for k, v in default_settings.items():
            if k not in d:
                d[k] = v
                changed = True
        if changed:
            self.save(d)

        self.data = d
        return d

    def save(self, data: dict):
        """
        Atomic-ish write: write tmp, then rename.
        (Micropython on some FS doesn't support atomic rename perfectly, but this helps.)
        """
        try:
            with open(TMP_PATH, "w") as f:
                ujson.dump(data, f)
                f.flush()
            try:
                os.remove(self.path)
            except Exception:
                pass
            os.rename(TMP_PATH, self.path)
        except Exception as e:
            # Best effort: if rename fails, try direct write
            try:
                with open(self.path, "w") as f:
                    ujson.dump(data, f)
                    f.flush()
            except Exception:
                pass

    def get(self, key, default=None):
        if not isinstance(self.data, dict):
            return default
        return self.data.get(key, default)

    def set(self, key, value):
        if not isinstance(self.data, dict):
            self.data = {}
        self.data[key] = value

    def commit(self):
        if isinstance(self.data, dict):
            self.save(self.data)
