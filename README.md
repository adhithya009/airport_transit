# Dynamic Headway Optimization in Automated Airport Transit

This repo simulates a single-corridor automated airport transit system and compares two dispatch strategies under stochastic (Poisson) passenger demand:

- **Fixed interval dispatch**: launch every \(X\) seconds
- **Dynamic safety headway dispatch**: launch only when the lead vehicle is beyond a safety distance

## Quick start

```bash
python -m venv .venv
.\.venv\Scripts\activate
pip install -r requirements.txt
python main.py
```

Outputs (plots + summary JSON) are written to `outputs/`.

## Model (short report-style)

### Assumptions

- Single straight corridor, one boarding terminal (A) and one destination (B)
- No intermediate stations; passengers only board at A and alight at B
- Vehicles follow a simple longitudinal control law with an emergency safety constraint
- Passenger arrivals at A follow a Poisson process (exponential inter-arrivals)

### Vehicle kinematics (discrete time)

At time step \(\Delta t\):

\[
x \leftarrow x + v\Delta t + 0.5a\Delta t^2,\quad
v \leftarrow \max(0, \min(v + a\Delta t, v_{\max}))
\]

### Safety headway

The dispatch and following constraint uses:

\[
d_{safe}(v) = \frac{v^2}{2|a_{brake}|} + buffer
\]

### Passenger demand and waiting time

Passengers arrive with exponential inter-arrival times. Waiting time is computed exactly by tracking arrival timestamps and summing \(t_{board} - t_{arrive}\) for each boarded passenger.

### Metrics

- Total passengers transported
- Average passenger wait time
- Maximum queue length
- Vehicle throughput (vehicles/hour)
- Corridor utilization (fraction of time with \(\ge 1\) vehicle in corridor)

## Configuration

Edit parameters in `airport_transit_sim/config.py`, then re-run `python main.py`.

