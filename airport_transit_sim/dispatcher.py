from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from .vehicle import safe_distance_m


class DispatchStrategy(str, Enum):
    FIXED_INTERVAL = "fixed_interval"
    DYNAMIC_HEADWAY = "dynamic_headway"


@dataclass
class FixedIntervalDispatcher:
    interval_s: float
    next_dispatch_s: float = 0.0

    def initialize(self, t0_s: float = 0.0) -> None:
        self.next_dispatch_s = t0_s

    def should_dispatch(self, t_s: float) -> bool:
        return t_s >= self.next_dispatch_s

    def on_dispatch(self, t_s: float) -> None:
        # schedule next
        self.next_dispatch_s = t_s + self.interval_s


@dataclass
class DynamicHeadwayDispatcher:
    max_brake_mps2: float
    buffer_m: float
    assumed_speed_mps: float

    def initialize(self, t0_s: float = 0.0) -> None:
        _ = t0_s

    def should_dispatch(self, active_vehicle_positions_m: list[float], closest_vehicle_speed_mps: float | None) -> bool:
        if not active_vehicle_positions_m:
            return True
        closest_pos = min(active_vehicle_positions_m)
        v_lead = 0.0 if closest_vehicle_speed_mps is None else closest_vehicle_speed_mps
        v = max(v_lead, self.assumed_speed_mps)
        required = safe_distance_m(v, self.max_brake_mps2, self.buffer_m)
        return closest_pos >= required

    def on_dispatch(self, t_s: float) -> None:
        _ = t_s

