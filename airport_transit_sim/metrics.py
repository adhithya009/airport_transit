from __future__ import annotations

from dataclasses import asdict, dataclass


@dataclass
class RunMetrics:
    strategy: str
    seed: int

    passengers_transported: int
    avg_wait_time_s: float
    max_queue_len: int

    vehicles_dispatched: int
    vehicles_arrived: int
    vehicle_throughput_per_hr: float

    corridor_occupied_fraction: float

    def to_dict(self) -> dict:
        return asdict(self)


def mean_std(values: list[float]) -> tuple[float, float]:
    if not values:
        return 0.0, 0.0
    m = sum(values) / len(values)
    var = sum((x - m) * (x - m) for x in values) / max(1, (len(values) - 1))
    return m, var**0.5

