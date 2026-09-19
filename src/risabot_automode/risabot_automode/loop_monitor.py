"""Small helper for loop jitter and overrun monitoring."""

import time
from typing import Dict


class LoopMonitor:
    """Tracks loop timing stats and computes periodic snapshots."""

    def __init__(self, loop_name: str, target_hz: float, overrun_ratio: float = 1.5) -> None:
        self.loop_name = loop_name
        self.target_hz = float(target_hz) if target_hz > 0 else 1.0
        self.expected_dt = 1.0 / self.target_hz
        self.overrun_dt = self.expected_dt * float(overrun_ratio)
        self.last_t = 0.0
        self.reset()

    def reset(self) -> None:
        """Reset accumulated counters."""
        self.count = 0
        self.sum_dt = 0.0
        self.max_dt = 0.0
        self.overruns = 0

    def tick(self) -> None:
        """Record one iteration timing."""
        now = time.monotonic()
        if self.last_t > 0.0:
            dt = now - self.last_t
            self.count += 1
            self.sum_dt += dt
            if dt > self.max_dt:
                self.max_dt = dt
            if dt > self.overrun_dt:
                self.overruns += 1
        self.last_t = now

    def snapshot(self) -> Dict[str, float]:
        """Return stats since last reset and reset counters."""
        avg_dt = (self.sum_dt / self.count) if self.count > 0 else 0.0
        avg_hz = (1.0 / avg_dt) if avg_dt > 0 else 0.0
        data = {
            'loop': self.loop_name,
            'target_hz': round(self.target_hz, 3),
            'avg_hz': round(avg_hz, 3),
            'max_dt_ms': round(self.max_dt * 1000.0, 3),
            'overruns': int(self.overruns),
            'samples': int(self.count),
        }
        self.reset()
        return data
