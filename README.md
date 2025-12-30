# UAV Strategic Deconfliction System

A comprehensive system for validating autonomous drone missions in shared airspace by detecting spatio-temporal conflicts with concurrent flight operations.

## Overview

This project implements a deconfliction service that:
- Validates waypoint missions for flight safety.
- Detects conflicts in both space and time.
- Provides detailed conflict reporting with locations and times.
- Visualizes trajectories in 2D, 3D, and spatio-temporal dimensions.
- Includes a scalability plan for 10,000+ concurrent drone missions.
- Provides a comprehensive test suite and documentation.[web:1]

**Assignment:** FlytBase Robotics Assignment 2025  
**Student:** B.Tech Robotics & Automation, 4th Year.[web:1]

---

## Quick Start

### Prerequisites

```
Python 3.8+
pip install numpy matplotlib
```

Or, using the provided file:

```
pip install -r requirements.txt
```

### Installation

```
# Clone or copy the project
cd flytbase_uav_deconfliction

# (Optional, if you have the file)
pip install -r requirements.txt
```

### Running the System

```
# Execute all test scenarios
python uav_deconfliction_complete.py
```

This will:

- Print deconfliction reports for each scenario.
- Generate visualization images:
  - `scenario_1_2d.png`, `scenario_1_3d.png`
  - `scenario_2_2d.png`, `scenario_2_3d.png`, `scenario_2_spatiotemporal.png`
  - `scenario_3_3d.png`

---

## System Architecture

### Core Components

#### 1. Data Structures (`Waypoint`, `DroneTrajectory`)

```
from uav_deconfliction_complete import Waypoint, DroneTrajectory

# Define a waypoint in 3D space
waypoint = Waypoint(x=50, y=50, z=10)  # meters

# Create a complete mission trajectory
mission = DroneTrajectory(
    drone_id="DRONE-A",
    waypoints=[
        Waypoint(0, 0, 10),
        Waypoint(50, 50, 10),
        Waypoint(100, 100, 10),
    ],
    arrival_times=,  # seconds[1][2]
    safety_buffer=15.0,         # meters
)
```

#### 2. Deconfliction Service

```
from uav_deconfliction_complete import DeconflictionService

# Initialize the service
service = DeconflictionService(time_resolution=0.1)

# Check mission against other active flights
status, conflicts = service.check_mission_safety(
    primary_mission=mission,
    other_missions=[other_drone1, other_drone2],
    verbose=True,  # Print detailed report
)

if status == "CLEAR":
    print("Mission approved for execution")
else:
    print(f"{len(conflicts)} conflicts detected")
    for conflict in conflicts:
        print(conflict)
```

#### 3. Visualization Module

```
from uav_deconfliction_complete import TrajectoryVisualizer

# 2D Visualization (XY plane)
TrajectoryVisualizer.plot_2d_trajectories(
    primary_mission,
    other_missions,
    conflicts,
    save_path="trajectory_2d.png",
)

# 3D Visualization (with altitude)
TrajectoryVisualizer.plot_3d_trajectories(
    primary_mission,
    other_missions,
    conflicts,
    save_path="trajectory_3d.png",
)

# Spatio-temporal Analysis (distance vs time)
TrajectoryVisualizer.plot_spatiotemporal(
    primary_mission,
    other_missions,
    conflicts,
    save_path="distance_analysis.png",
)
```

---

## Usage Examples

### Example 1: Simple Safety Check

```
from uav_deconfliction_complete import (
    DroneTrajectory,
    Waypoint,
    DeconflictionService,
)

# Primary mission
primary = DroneTrajectory(
    drone_id="MY_DRONE",
    waypoints=[
        Waypoint(0, 0, 10),
        Waypoint(100, 100, 10),
    ],
    arrival_times=,[3]
    safety_buffer=20.0,
)

# Other active missions
other = [
    DroneTrajectory(
        drone_id="OTHER_DRONE",
        waypoints=[
            Waypoint(100, 0, 10),
            Waypoint(0, 100, 10),
        ],
        arrival_times=,[3]
        safety_buffer=20.0,
    )
]

# Run deconfliction check
service = DeconflictionService()
status, conflicts = service.check_mission_safety(primary, other)

print(f"Status: {status}")
if conflicts:
    print(f"Conflicts: {len(conflicts)}")
    for c in conflicts:
        print(f"  - At {c.time:.2f}s: {c.min_distance:.2f}m separation")
```

### Example 2: 3D Mission with Altitude

```
from uav_deconfliction_complete import (
    DroneTrajectory,
    Waypoint,
    DeconflictionService,
    TrajectoryVisualizer,
)

# Mission with altitude changes
altitude_mission = DroneTrajectory(
    drone_id="CLIMBING_DRONE",
    waypoints=[
        Waypoint(0, 0, 5),      # Start at 5m altitude
        Waypoint(50, 50, 15),   # Climb to 15m
        Waypoint(100, 100, 30), # Climb to 30m
    ],
    arrival_times=,[4][5]
    safety_buffer=15.0,
)

# Other missions
other_drone1 = DroneTrajectory(
    drone_id="OTHER1",
    waypoints=[Waypoint(0, 100, 10), Waypoint(100, 0, 10)],
    arrival_times=,[5]
    safety_buffer=15.0,
)

other_drone2 = DroneTrajectory(
    drone_id="OTHER2",
    waypoints=[Waypoint(50, -50, 20), Waypoint(50, 150, 20)],
    arrival_times=,[6][5]
    safety_buffer=15.0,
)

# Check and visualize
service = DeconflictionService()
status, conflicts = service.check_mission_safety(
    altitude_mission,
    [other_drone1, other_drone2],
)

TrajectoryVisualizer.plot_3d_trajectories(
    altitude_mission,
    [other_drone1, other_drone2],
    conflicts,
    save_path="3d_conflict_analysis.png",
)
```

### Example 3: Batch Mission Validation

```
def validate_mission_batch(primary_mission, other_missions_list):
    """Check mission against multiple batches of other flights."""
    service = DeconflictionService()

    all_conflicts = []
    for batch in other_missions_list:
        status, conflicts = service.check_mission_safety(
            primary_mission,
            batch,
            verbose=False,
        )
        all_conflicts.extend(conflicts)

    print(f"Total conflicts across all batches: {len(all_conflicts)}")
    return all_conflicts
```

---

## Test Scenarios Included

### Scenario 1: Conflict-Free Mission

- Primary drone path far away from other missions.
- Expected: `CLEAR`.
- Outputs:
  - `scenario_1_2d.png`
  - `scenario_1_3d.png`.[web:1]

### Scenario 2: Mission with Conflicts

- Primary drone crosses paths with multiple drones.
- Expected: `CONFLICT DETECTED` with multiple conflicts.
- Outputs:
  - `scenario_2_2d.png`
  - `scenario_2_3d.png`
  - `scenario_2_spatiotemporal.png`.[web:1]

### Scenario 3: 3D Altitude-Varying Mission

- Drones at different altitudes with overlapping XY paths.
- Demonstrates 3D spatio-temporal reasoning.
- Output:
  - `scenario_3_3d.png`.[web:1]

---

## Algorithm Details

### Spatio-Temporal Conflict Detection

**Idea:**  
Sample time within overlapping mission windows; interpolate positions; compute distances; check against safety threshold.

**Steps:**
1. Compute temporal overlap between primary and each other mission.
2. For each overlapping interval, sample \( t \) with step `time_resolution` (default 0.1 s).
3. For each \( t \):
   - Interpolate primary position.
   - Interpolate other drone position.
   - Compute 3D Euclidean distance.
4. Track the minimum distance per pair.
5. If minimum distance < safety threshold → record a `Conflict` object.[web:8]

**Complexity:**
- Time: \( O(n \cdot k) \) per query, where:
  - \( n \): number of other missions
  - \( k \): number of time samples (duration / time_resolution)
- Space: \( O(n) \) for conflicts + trajectories.[web:8]

---

## Configuration & Tuning

### Time Resolution

```
service = DeconflictionService(time_resolution=0.1)
```

- 0.1 s: Good balance (default).
- Lower (0.05): More accurate, slower.
- Higher (0.5): Faster, may miss very brief conflicts.[web:8]

### Safety Buffer

```
mission = DroneTrajectory(
    drone_id="DRONE-A",
    waypoints=[...],
    arrival_times=[...],
    safety_buffer=25.0,  # meters
)
```

- Typical values: 10–25 m.
- Use higher values for high-density or high-uncertainty environments.[web:11]

---

## Scalability (High-Level)

To scale to tens of thousands of drones (as per assignment requirement):[web:1]

- Use **spatial partitioning** (3D grid cells) to reduce pairwise checks.
- Use **time binning** (5-min windows) to avoid checking missions that cannot overlap.
- Deploy as a **distributed microservice** with:
  - Kafka (trajectory updates),
  - Flink/Spark Streaming (conflict detection),
  - Redis (recent state cache),
  - PostgreSQL/TimescaleDB (conflict logs).

These details are elaborated in `REFLECTION_AND_DESIGN.md`.[web:1]

---

## Limitations & Future Work

Current limitations:
- Deterministic trajectories (no wind/weather uncertainty).
- No dynamic re-routing or conflict resolution (detection only).
- No explicit geofencing / restricted airspace integration.
- Single-process implementation (scalability via architecture doc, not yet coded).[web:1][web:8]

Possible extensions:
- Probabilistic conflict detection under uncertainty.
- Online re-routing and path optimization.
- Integration with real UTM systems (e.g., NASA/FAA frameworks).
- Real-time monitoring and continuous trajectory updates.[web:31]

---

## References

- FlytBase Robotics Assignment PDF for detailed requirements.[file:1]
- Python `unittest` documentation for test structure.[web:30]
- Research on trajectory conflict detection and deconfliction for UAV and ATM systems.[web:8][web:31]

---

## Support

If you:
- Want to just **run** and see outputs: read `QUICK_START.md`.
- Want to **understand design**: read `REFLECTION_AND_DESIGN.md`.
- Want to **modify or extend**: start from this `README.md` and code docstrings in `uav_deconfliction_complete.py`.
```

[1](https://www.codac.io/manual/02-variables/02-var-dynamic.html)
[2](https://docs.python.org/3/library/unittest.html)
[3](https://www.sciencedirect.com/science/article/pii/S0951832025003795)
[4](https://www.bragitoff.com/2020/10/3d-trajectory-animated-using-matplotlib-python/)
[5](https://github.com/AbhiFutureTech/UAV-Strategic-Deconfliction-System)
[6](https://www.mathworks.com/help/nav/ref/waypointtrajectory-system-object.html)

[7](https://ppl-ai-file-upload.s3.amazonaws.com/web/direct-files/attachments/84521017/26193644-c626-4cda-8ca7-b01ec2c3da5f/FlytBase-Robotics-Assignment-2025.pdf)
