"""Single inference worker with source-time selection and one pending frame."""
import math
from threading import Condition, Thread


class LatestFrameWorker:
    def __init__(self, callback, target_hz, on_error):
        if not math.isfinite(target_hz) or target_hz <= 0:
            raise ValueError('target_hz must be finite and positive')
        self.callback, self.on_error = callback, on_error
        self.period_ns = max(1, round(1e9 / target_hz))
        self.condition = Condition()
        self.pending = None
        self.stopping = False
        self.origin = self.last_stamp = None
        self.last_slot = -1
        self.replaced = 0
        self.received = self.selected = self.completed = self.errors = 0
        self.thread = Thread(target=self._run, name='m2h-inference', daemon=False)
        self.thread.start()

    def submit(self, stamp_ns, value):
        with self.condition:
            if self.stopping:
                return False
            self.received += 1
            if self.last_stamp is not None and stamp_ns == self.last_stamp:
                return False
            if self.origin is None or stamp_ns < self.last_stamp:
                self.origin, self.last_slot = stamp_ns, -1
                self.pending = None
            self.last_stamp = stamp_ns
            slot = (stamp_ns-self.origin) // self.period_ns
            if slot <= self.last_slot:
                return False
            self.last_slot = slot
            if self.pending is not None:
                self.replaced += 1
            self.pending = value
            self.selected += 1
            self.condition.notify()
            return True

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.stopping or self.pending is not None)
                if self.stopping:
                    return
                value, self.pending = self.pending, None
            try:
                self.callback(*value)
            except Exception as exc:
                with self.condition:
                    self.errors += 1
                self.on_error(exc)
            finally:
                with self.condition:
                    self.completed += 1

    def statistics(self):
        """Callback counts, not a claim that every callback published depth."""
        with self.condition:
            return dict(received=self.received, selected=self.selected,
                        completed=self.completed, replaced=self.replaced, errors=self.errors)

    def close(self):
        with self.condition:
            self.stopping = True
            self.pending = None
            self.condition.notify_all()
        self.thread.join()
