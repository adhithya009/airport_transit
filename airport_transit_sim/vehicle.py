from __future__ import annotations

from dataclasses import dataclass


def safe_distance_m(v_mps: float, max_brake_mps2: float, buffer_m: float) -> float:
    # d = v^2 / (2|a|) + buffer
    if max_brake_mps2 <= 0:
        raise ValueError("max_brake_mps2 must be > 0")
    return (v_mps * v_mps) / (2.0 * max_brake_mps2) + buffer_m


@dataclass
class Vehicle:
    vid: int
    position_m: float = 0.0
    velocity_mps: float = 0.0
    acceleration_mps2: float = 0.0
    dispatch_time_s: float = 0.0
    arrival_time_s: float | None = None
    passengers_onboard: int = 0

    def step(self, dt_s: float, max_speed_mps: float) -> None:
        x = self.position_m + self.velocity_mps * dt_s + 0.5 * self.acceleration_mps2 * dt_s * dt_s
        v = self.velocity_mps + self.acceleration_mps2 * dt_s
        if v < 0:
            v = 0.0
        if v > max_speed_mps:
            v = max_speed_mps
        self.position_m = x
        self.velocity_mps = v

