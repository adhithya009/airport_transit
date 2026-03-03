from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field

import numpy as np


@dataclass
class PassengerQueue:
    arrival_rate_per_s: float
    rng: np.random.Generator

    # store individual arrival timestamps for exact waiting time calculation
    arrivals_s: deque[float] = field(default_factory=deque)
    next_arrival_s: float = 0.0

    total_arrived: int = 0
    total_boarded: int = 0
    total_wait_time_s: float = 0.0
    max_queue_len: int = 0

    def initialize(self, t0_s: float = 0.0) -> None:
        self.arrivals_s.clear()
        self.next_arrival_s = t0_s + self._sample_interarrival_s()
        self.total_arrived = 0
        self.total_boarded = 0
        self.total_wait_time_s = 0.0
        self.max_queue_len = 0

    def _sample_interarrival_s(self) -> float:
        if self.arrival_rate_per_s <= 0:
            return float("inf")
        # Exponential inter-arrival with mean 1/lambda
        return float(self.rng.exponential(scale=1.0 / self.arrival_rate_per_s))

    def update_arrivals(self, t_s: float) -> None:
        # generate all arrivals up to current time
        while self.next_arrival_s <= t_s:
            self.arrivals_s.append(self.next_arrival_s)
            self.total_arrived += 1
            self.next_arrival_s += self._sample_interarrival_s()
        qlen = len(self.arrivals_s)
        if qlen > self.max_queue_len:
            self.max_queue_len = qlen

    @property
    def queue_length(self) -> int:
        return len(self.arrivals_s)

    def board(self, t_board_s: float, capacity: int) -> int:
        n = min(capacity, len(self.arrivals_s))
        for _ in range(n):
            t_arrive = self.arrivals_s.popleft()
            self.total_wait_time_s += (t_board_s - t_arrive)
        self.total_boarded += n
        return n

