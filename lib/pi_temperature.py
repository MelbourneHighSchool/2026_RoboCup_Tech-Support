"""Sample the Pi's SoC temperature without doing sensor I/O in the control loop."""

import threading
from pathlib import Path


class PiTemperature:
    def __init__(self, path="/sys/class/thermal/thermal_zone0/temp", interval_s=1.0):
        self.path = Path(path)
        self.interval_s = interval_s
        self._value = None
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run, name="pi-temperature", daemon=True,
        )

    @property
    def celsius(self):
        return self._value

    def start(self):
        self._thread.start()

    def _sample(self):
        try:
            self._value = int(self.path.read_text(encoding="ascii").strip()) / 1000.0
        except (OSError, ValueError, UnicodeError):
            # Missing/unreadable sensors must never prevent recording or driving.
            self._value = None

    def _run(self):
        while not self._stop.is_set():
            self._sample()
            self._stop.wait(self.interval_s)

    def stop(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
