"""
Samuha TVF-ORCA Hybrid Flight Control Package

Provides:
- Drone: SITL (simulated) drone control via MAVSDK
- HITLDrone: Hardware-In-The-Loop drone control via serial/USB
- Telemetry estimation, collision avoidance, and swarm logging
"""

from .config import (
    MAVSDK_SERVER_BIN,
    SDK_PORT_OFFSET,
    UPDATE_RATE_HZ,
    DT,
    TAKEOFF_ALTITUDE,
    MAX_HEIGHT_M,
    MAX_SPEED_XY,
    MAX_SPEED_Z,
    MAX_ACCEL_XY,
    MAX_ACCEL_Z,
    GOAL_GAIN,
    GOAL_TOLERANCE,
    REPULSION_GAIN,
    TANGENTIAL_GAIN,
    SAFE_RADIUS,
    INFLUENCE_RADIUS,
)

from .drone import Drone
from .hitl_drone import HITLDrone, HITLConfig, shared_swarm_telemetry
from .kalman import AsyncKalmanNED
from .collision_avoidance import hybrid_velocity
from .logger import SwarmLogger, StateRow, EventRow

__all__ = [
    # Configuration
    "MAVSDK_SERVER_BIN",
    "SDK_PORT_OFFSET",
    "UPDATE_RATE_HZ",
    "DT",
    "TAKEOFF_ALTITUDE",
    "MAX_HEIGHT_M",
    "MAX_SPEED_XY",
    "MAX_SPEED_Z",
    "MAX_ACCEL_XY",
    "MAX_ACCEL_Z",
    "GOAL_GAIN",
    "GOAL_TOLERANCE",
    "REPULSION_GAIN",
    "TANGENTIAL_GAIN",
    "SAFE_RADIUS",
    "INFLUENCE_RADIUS",
    # Classes
    "Drone",
    "HITLDrone",
    "HITLConfig",
    "AsyncKalmanNED",
    "SwarmLogger",
    "StateRow",
    "EventRow",
    # Functions & Data
    "hybrid_velocity",
    "shared_swarm_telemetry",
]
