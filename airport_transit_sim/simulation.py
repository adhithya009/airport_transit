from __future__ import annotations

import json
import os
from dataclasses import dataclass

import matplotlib.pyplot as plt
import numpy as np

from .config import SimConfig
from .dispatcher import (
    DispatchStrategy,
    DynamicHeadwayDispatcher,
    FixedIntervalDispatcher,
)
from .metrics import RunMetrics, mean_std
from .passenger_model import PassengerQueue
from .vehicle import Vehicle, safe_distance_m


@dataclass
class RunLog:
    t_s: list[float]
    queue_len: list[int]


def _ensure_outputs_dir(outputs_dir: str) -> None:
    os.makedirs(outputs_dir, exist_ok=True)


def _update_vehicle_controls(cfg: SimConfig, vehicles: list[Vehicle]) -> None:
    # Simple car-following:
    # - if too close to leader: brake hard
    # - else accelerate up to MAX_SPEED
    # Sort from front-most (leader) to back-most (closest to origin)
    vehicles.sort(key=lambda v: v.position_m, reverse=True)
    for i, veh in enumerate(vehicles):
        if i == 0:
            # lead vehicle: free flow
            if veh.velocity_mps < cfg.MAX_SPEED_MPS:
                veh.acceleration_mps2 = cfg.MAX_ACCEL_MPS2
            else:
                veh.acceleration_mps2 = 0.0
            continue

        leader = vehicles[i - 1]  # vehicle ahead (larger position)
        gap = leader.position_m - veh.position_m
        required = safe_distance_m(veh.velocity_mps, cfg.MAX_BRAKE_MPS2, cfg.SAFETY_BUFFER_M)
        if gap < required:
            veh.acceleration_mps2 = -cfg.MAX_BRAKE_MPS2
        else:
            if veh.velocity_mps < cfg.MAX_SPEED_MPS:
                veh.acceleration_mps2 = cfg.MAX_ACCEL_MPS2
            else:
                veh.acceleration_mps2 = 0.0


def run_simulation(
    cfg: SimConfig,
    strategy: DispatchStrategy,
    seed: int,
    record_queue_trace: bool = False,
) -> tuple[RunMetrics, RunLog | None]:
    rng = np.random.default_rng(seed)
    pq = PassengerQueue(arrival_rate_per_s=cfg.ARRIVAL_RATE_PAX_PER_S, rng=rng)
    pq.initialize(0.0)

    fixed = FixedIntervalDispatcher(interval_s=cfg.FIXED_INTERVAL_S)
    dyn = DynamicHeadwayDispatcher(
        max_brake_mps2=cfg.MAX_BRAKE_MPS2,
        buffer_m=cfg.SAFETY_BUFFER_M,
        assumed_speed_mps=cfg.MAX_SPEED_MPS,
    )
    fixed.initialize(0.0)
    dyn.initialize(0.0)

    vehicles: list[Vehicle] = []
    next_vid = 1
    vehicles_dispatched = 0
    vehicles_arrived = 0
    passengers_transported = 0

    occupied_steps = 0
    total_steps = int(np.ceil(cfg.HORIZON_S / cfg.TIME_STEP_S))

    log = RunLog(t_s=[], queue_len=[]) if record_queue_trace else None

    t_s = 0.0
    for _ in range(total_steps + 1):
        pq.update_arrivals(t_s)

        if vehicles:
            occupied_steps += 1

        # dispatch decision
        can_dispatch = False
        if strategy == DispatchStrategy.FIXED_INTERVAL:
            can_dispatch = fixed.should_dispatch(t_s)
        elif strategy == DispatchStrategy.DYNAMIC_HEADWAY:
            active_positions = [v.position_m for v in vehicles]
            closest_speed = None
            if vehicles:
                closest = min(vehicles, key=lambda v: v.position_m)
                closest_speed = closest.velocity_mps
            can_dispatch = dyn.should_dispatch(active_positions, closest_speed)
        else:
            raise ValueError(f"Unknown strategy: {strategy}")

        if can_dispatch:
            boarded = pq.board(t_s, cfg.VEHICLE_CAPACITY)
            v = Vehicle(
                vid=next_vid,
                position_m=0.0,
                velocity_mps=0.0,
                acceleration_mps2=cfg.MAX_ACCEL_MPS2,
                dispatch_time_s=t_s,
                passengers_onboard=boarded,
            )
            vehicles.append(v)
            next_vid += 1
            vehicles_dispatched += 1
            if strategy == DispatchStrategy.FIXED_INTERVAL:
                fixed.on_dispatch(t_s)
            else:
                dyn.on_dispatch(t_s)

        # update controls then integrate dynamics
        if vehicles:
            _update_vehicle_controls(cfg, vehicles)
            for v in vehicles:
                v.step(cfg.TIME_STEP_S, cfg.MAX_SPEED_MPS)

        # check arrivals and remove completed vehicles
        still_active: list[Vehicle] = []
        for v in vehicles:
            if v.position_m >= cfg.CORRIDOR_LENGTH_M:
                v.arrival_time_s = t_s
                vehicles_arrived += 1
                passengers_transported += v.passengers_onboard
            else:
                still_active.append(v)
        vehicles = still_active

        if log is not None:
            log.t_s.append(t_s)
            log.queue_len.append(pq.queue_length)

        t_s += cfg.TIME_STEP_S
        if t_s > cfg.HORIZON_S + 1e-9:
            break

    avg_wait = (pq.total_wait_time_s / pq.total_boarded) if pq.total_boarded > 0 else 0.0
    throughput = vehicles_arrived * (3600.0 / cfg.HORIZON_S) if cfg.HORIZON_S > 0 else 0.0
    occupied_fraction = occupied_steps / max(1, (total_steps + 1))

    metrics = RunMetrics(
        strategy=strategy.value,
        seed=seed,
        passengers_transported=passengers_transported,
        avg_wait_time_s=avg_wait,
        max_queue_len=pq.max_queue_len,
        vehicles_dispatched=vehicles_dispatched,
        vehicles_arrived=vehicles_arrived,
        vehicle_throughput_per_hr=throughput,
        corridor_occupied_fraction=occupied_fraction,
    )
    return metrics, log


def run_experiments(cfg: SimConfig, outputs_dir: str = "outputs") -> dict:
    _ensure_outputs_dir(outputs_dir)

    strategies = [DispatchStrategy.FIXED_INTERVAL, DispatchStrategy.DYNAMIC_HEADWAY]

    all_runs: list[RunMetrics] = []
    queue_trace_example: RunLog | None = None
    for strat in strategies:
        for i in range(cfg.N_RUNS):
            seed = cfg.BASE_SEED + i
            metrics, log = run_simulation(
                cfg,
                strat,
                seed=seed,
                record_queue_trace=(strat == DispatchStrategy.DYNAMIC_HEADWAY and i == 0),
            )
            all_runs.append(metrics)
            if log is not None:
                queue_trace_example = log

    # aggregate
    summary: dict = {"config": cfg.__dict__, "strategies": {}}
    for strat in strategies:
        runs = [m for m in all_runs if m.strategy == strat.value]
        summary["strategies"][strat.value] = {
            "n_runs": len(runs),
            "passengers_transported": {
                "mean": mean_std([m.passengers_transported for m in runs])[0],
                "std": mean_std([float(m.passengers_transported) for m in runs])[1],
            },
            "avg_wait_time_s": {
                "mean": mean_std([m.avg_wait_time_s for m in runs])[0],
                "std": mean_std([m.avg_wait_time_s for m in runs])[1],
            },
            "max_queue_len": {
                "mean": mean_std([float(m.max_queue_len) for m in runs])[0],
                "std": mean_std([float(m.max_queue_len) for m in runs])[1],
            },
            "vehicle_throughput_per_hr": {
                "mean": mean_std([m.vehicle_throughput_per_hr for m in runs])[0],
                "std": mean_std([m.vehicle_throughput_per_hr for m in runs])[1],
            },
            "corridor_occupied_fraction": {
                "mean": mean_std([m.corridor_occupied_fraction for m in runs])[0],
                "std": mean_std([m.corridor_occupied_fraction for m in runs])[1],
            },
        }

    with open(os.path.join(outputs_dir, "summary.json"), "w", encoding="utf-8") as f:
        json.dump(summary, f, indent=2)

    _plot_bars(summary, outputs_dir)
    if queue_trace_example is not None:
        _plot_queue_trace(queue_trace_example, outputs_dir)

    return summary


def _plot_bars(summary: dict, outputs_dir: str) -> None:
    labels = ["fixed_interval", "dynamic_headway"]
    strategies = summary["strategies"]

    def means(key: str) -> list[float]:
        return [float(strategies[s][key]["mean"]) for s in labels]

    # throughput bar
    plt.figure(figsize=(6, 4))
    plt.bar(labels, means("vehicle_throughput_per_hr"))
    plt.ylabel("Vehicles / hour")
    plt.title("Vehicle throughput comparison")
    plt.tight_layout()
    plt.savefig(os.path.join(outputs_dir, "throughput_comparison.png"), dpi=160)
    plt.close()

    # wait time bar
    plt.figure(figsize=(6, 4))
    plt.bar(labels, means("avg_wait_time_s"))
    plt.ylabel("Avg passenger wait (s)")
    plt.title("Average passenger wait time")
    plt.tight_layout()
    plt.savefig(os.path.join(outputs_dir, "wait_time_comparison.png"), dpi=160)
    plt.close()


def _plot_queue_trace(log: RunLog, outputs_dir: str) -> None:
    plt.figure(figsize=(8, 4))
    plt.plot(log.t_s, log.queue_len, linewidth=1.5)
    plt.xlabel("Time (s)")
    plt.ylabel("Queue length (pax)")
    plt.title("Queue length vs time (example run)")
    plt.tight_layout()
    plt.savefig(os.path.join(outputs_dir, "queue_trace.png"), dpi=160)
    plt.close()

