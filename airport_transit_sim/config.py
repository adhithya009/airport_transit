from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class SimConfig:
    # Corridor / physics
    CORRIDOR_LENGTH_M: float = 5000.0
    MAX_SPEED_MPS: float = 15.0  # ~54 km/h
    MAX_ACCEL_MPS2: float = 1.0
    MAX_BRAKE_MPS2: float = 2.0  # magnitude, used as |a_brake|
    SAFETY_BUFFER_M: float = 10.0
    TIME_STEP_S: float = 0.5

    # Vehicles / operations
    VEHICLE_CAPACITY: int = 40

    # Demand (Poisson): ARRIVAL_RATE_PAX_PER_S = lambda
    ARRIVAL_RATE_PAX_PER_S: float = 0.6  # 0.6 pax/s = 2160 pax/hr

    # Dispatch policies
    FIXED_INTERVAL_S: float = 60.0

    # Simulation horizon
    HORIZON_S: float = 3600.0

    # RNG / experiments
    N_RUNS: int = 30
    BASE_SEED: int = 12345

