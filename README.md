# Samuha

<video src="Samuha_TVF_ORCA/demo.mp4" controls muted loop width="640">
Your browser does not support the video tag. You can still view the demo in `Samuha_TVF_ORCA/demo.mp4`.
</video>

## Overview

This repository collects swarm simulation work for multi-drone testing on Ubuntu 22.04 with Gazebo Classic, PX4 v1.16 SITL, and MAVSDK. The main goal is to validate and compare reactive swarm algorithms in simulation before moving to real-world testing.

The development path is:
1. `VF` — baseline velocity field swarm motion
2. `TVF` — tangential vector field extension
3. `TVF+ORCA` — hybrid safety layer combining tangential flow with ORCA-style emergency avoidance

ROS 2 Humble is also used in the simulation environment to keep the setup future-compatible with upcoming PX4 ROS interface support.

## Repository structure

- `Samuha_VF/` — baseline swarm control with velocity field logic
- `Samuha_TVF/` — tangential vector field swarm control
- `Samuha_TVF_ORCA/` — TVF + ORCA hybrid collision avoidance
- `sitl_multiple_ros_run.sh` — custom script to spawn multiple PX4 SITL vehicles in Gazebo Classic; it is expected to be located under `/PX4-Autopilot/Tools/simulation/gazebo-classic/` for the PX4/Gazebo workflow
- `MAVSDK_path_find.py` — helper script to locate the `mavsdk_server` binary
- `requirements.txt` — currently empty; MAVSDK is installed via pip manually

### Expected repository layout

```text
Samuha/
├── README.md
├── MAVSDK_path_find.py
├── requirements.txt
├── Samuha_VF/
│   ├── VF.py
│   └── src/
├── Samuha_TVF/
│   ├── tvf_main.py
│   └── src/
├── Samuha_TVF_ORCA/
│   ├── TVF_ORCA_main.py
│   └── src/
└── Tools/
    └── simulation/
        └── gazebo-classic/
            └── sitl_multiple_ros_run.sh
```

## Algorithm summaries

### 1. VF (Velocity Field)

**Logic**
- Each drone uses a reactive potential field.
- Goal attraction vector pulls toward the assigned target.
- Neighbor repulsion pushes away from nearby drones.
- A weak tangential drift is added around obstacles to reduce head-on conflicts.

**Benefits**
- Simple and fast to compute.
- Good for low-density swarms and wide-open spaces.
- Easy to tune and inspect.

**Limits**
- Purely local; can get stuck in local minima.
- No explicit reciprocal avoidance, so close crossing scenarios can still cause oscillations or near-collisions.
- Sensitive to tuning when many drones operate in a shared airspace.

**Future improvements**
- Add neighbor velocity prediction.
- Add global planning or path guidance.
- Combine with a more robust safety layer such as ORCA.

### 2. TVF (Tangential Vector Field)

**Logic**
- Builds on VF by strengthening the tangential component.
- Attractive force toward the goal remains, but neighbor repulsion is shaped by a tangential flow field.
- This causes drones to circulate around each other instead of only pushing directly away.
- Telemetry noise and packet loss are simulated to test robustness.

**Benefits**
- Better flow through congested regions than basic VF.
- Encourages smoother, more natural avoidance behavior.
- Preserves progress toward the goal while avoiding local traps.

**Limits**
- Still reactive and local; dense crossings remain challenging.
- Can oscillate if neighbor geometry changes rapidly.
- Performance depends on accurate relative position estimates.

**Future improvements**
- Add adaptive influence radius based on speed and density.
- Integrate a predictive motion model for neighbors.
- Fuse with a higher-level planner for coordination.

### 3. TVF + ORCA Hybrid

**Logic**
- Uses the TVF field for nominal goal pursuit and coordinated avoidance.
- Adds an ORCA-inspired emergency layer for imminent collisions.
- The emergency layer applies a stronger reciprocal push when neighbors enter the safety radius.
- This hybrid approach keeps the smooth flow of TVF while improving close-quarter safety.

**Benefits**
- More resilient in dense swarm interactions.
- Handles emergency encounters better than TVF alone.
- Suitable for pre-IRL swarm stress testing and robustness evaluation.

**Limits**
- Still reactive; it does not solve every deadlock or high-density crossing case.
- Emergency pushes can delay goal arrival and increase path length.
- May need improved coordination when many drones are simultaneously in close proximity.

**Future improvements**
- Use predicted neighbor trajectories rather than current positions only.
- Add dynamic priority or role assignment to avoid mutual oscillation.
- Seamlessly merge with a PX4 ROS-native interface once that library becomes stable.

## Simulation and testing workflow

This repo is designed for simulation-first validation before IRL testing.

Testing order:
- Start with `Samuha_VF` and validate basic swarm behavior.
- Move to `Samuha_TVF` to evaluate tangential coordination.
- Finally use `Samuha_TVF_ORCA` to stress-test dense avoidance.

The custom script `sitl_multiple_ros_run.sh` was created to spawn and simulate multiple PX4 SITL drones in Gazebo Classic. It is the main entry point for multi-drone environment setup.

## Tools and setup used

- Ubuntu 22.04 LTS
- PX4 v1.16 SITL
- Gazebo Classic (PX4 SITL-compatible)
- MAVSDK installed via pip
- ROS 2 Humble sourced for Gazebo / future ROS compatibility
- Custom multi-drone spawn script: `sitl_multiple_ros_run.sh`
- MAVSDK server locator: `MAVSDK_path_find.py`

## Quick setup steps

1. Clone the repository:

```bash
git clone https://github.com/Amogh9341/Samuha.git
cd Samuha
```

2. Install Python and MAVSDK:

```bash
sudo apt update
sudo apt install -y python3 python3-pip python3-venv
python3 -m pip install --user mavsdk
```

3. Locate `mavsdk_server` if needed:

```bash
python3 MAVSDK_path_find.py
```

4. Update the `MAVSDK_SERVER_BIN` path in the config files if your environment differs:
- `Samuha_TVF/src/config.py`
- `Samuha_TVF_ORCA/src/config.py`

5. Build PX4 SITL if not already available. From a PX4 source tree:

```bash
git clone https://github.com/PX4/Firmware.git
cd Firmware
git checkout v1.16.0
make px4_sitl_default sitl_gazebo-classic
```

6. Place `sitl_multiple_ros_run.sh` under `Tools/simulation/gazebo-classic/` if it is not already there, then run the multi-drone Gazebo Classic launcher:

```bash
bash Tools/simulation/gazebo-classic/sitl_multiple_ros_run.sh -n 3 -m iris -w empty
```

7. Run one of the swarm controllers:

```bash
python3 Samuha_VF/VF.py
python3 Samuha_TVF/tvf_main.py
python3 Samuha_TVF_ORCA/TVF_ORCA_main.py
```

## How to use the repository

- `Samuha_VF/VF.py` — baseline algorithm
- `Samuha_TVF/tvf_main.py` — tangential vector field algorithm
- `Samuha_TVF_ORCA/TVF_ORCA_main.py` — hybrid TVF + ORCA algorithm

Use the above files to compare how the swarm behaves under each approach.

## Notes

- `ROS 2 Humble` is included to keep the simulation environment compatible with future PX4 ROS interface developments.
- The current design is simulation-first: it tests limitations, stability, and swarm behavior before any real drone deployment.
- The multi-drone script and logger are intended to support experimentation and analysis of swarm performance under telemetry impairment.

## Future directions

Potential next steps include:
- moving toward a PX4-native ROS2 interface once the PX4 ROS library matures,
- integrating global path planning,
- adding explicit velocity prediction or MPC,
- extending the emergency safety layer for tighter swarm corridors.

---

If you want, I can also add a second README section with exact command examples for a full multi-drone test case.