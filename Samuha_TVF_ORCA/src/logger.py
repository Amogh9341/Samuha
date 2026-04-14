# ===============================================================
# Swarm Logger Module - CSV-based Telemetry & Event Logging
# ===============================================================

import csv
import time
from pathlib import Path
from dataclasses import dataclass, asdict
from datetime import datetime


# ===============================================================
# DATA STRUCTURES
# ===============================================================


@dataclass
class StateRow:
    """
    Drone state snapshot at a single control loop iteration.
    Used to log all telemetry and command state for analysis.
    """

    # Timing
    timestamp: float  # Unix timestamp
    drone_id: int  # Port number for swarm identification

    # Position (NED, meters)
    north: float
    east: float
    down: float

    # Commanded velocity (m/s)
    cmd_vn: float
    cmd_ve: float
    cmd_vd: float

    # Raw desired velocity from collision avoidance (m/s)
    raw_vn: float
    raw_ve: float
    raw_vd: float

    # Vector to goal (meters)
    dx: float
    dy: float
    dz: float
    dist_goal: float

    # Neighbor info
    nearest_neighbor: float  # Distance to closest neighbor (m) or -1 if none
    num_neighbors: int  # Count of neighbors in awareness zone

    # Telemetry quality
    telemetry_age: float  # Seconds since last telemetry update

    # Flags (0 or 1)
    timeout_flag: int  # 1 if telemetry timed out this iteration
    packet_drop_flag: int  # 1 if packet was dropped this iteration
    burst_flag: int  # 1 if burst outage active this iteration


@dataclass
class EventRow:
    """
    Discrete event log entry (connection, arm, takeoff, error, etc).
    Lower frequency than StateRow.
    """

    timestamp: float  # Unix timestamp
    drone_id: int  # Port number
    event_type: str  # "connection", "arm", "takeoff", "offboard", "telemetry_timeout", "landing", "error"
    message: str  # Free-form details


# ===============================================================
# SWARM LOGGER CLASS
# ===============================================================


class SwarmLogger:
    """
    Production-ready CSV logger for multi-drone swarm experiments.

    Creates a timestamped run folder and writes:
    - states.csv: Control loop telemetry snapshots
    - events.csv: Discrete flight events

    Designed for async operation with periodic flushing.
    Thread-safe with reference counting for multi-drone shutdown.
    """

    # Class-level reference counter for shared logger
    _active_drones = 0

    def __init__(self, log_dir: str = "logs", run_name: str = None):
        """
        Initialize logger and create folder structure.

        Args:
            log_dir: Parent directory for all runs (default: "logs")
            run_name: Custom run folder name. If None, uses timestamp.
                     Format example: "run_20260414_153045"
        """
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(exist_ok=True)

        # Create timestamped run folder
        if run_name is None:
            now_str = datetime.now().strftime("%Y%m%d_%H%M%S")
            run_name = f"run_{now_str}"

        self.run_folder = self.log_dir / run_name
        self.run_folder.mkdir(exist_ok=True)

        # File paths
        self.states_file = self.run_folder / "states.csv"
        self.events_file = self.run_folder / "events.csv"

        # File handles (opened in append mode to support multiple drones)
        self.states_handle = None
        self.events_handle = None

        # CSV writers
        self.states_writer = None
        self.events_writer = None

        # Flush counters
        self.state_count = 0
        self.flush_interval = 50  # Flush states after N rows

        # Initialize CSV files with headers
        self._init_states_csv()
        self._init_events_csv()

    def _init_states_csv(self):
        """Create states.csv and write header."""
        # Check if file exists to avoid rewriting headers
        file_exists = self.states_file.exists()

        self.states_handle = open(self.states_file, "a", newline="")
        self.states_writer = csv.DictWriter(
            self.states_handle, fieldnames=[f.name for f in StateRow.__dataclass_fields__.values()]
        )

        if not file_exists:
            self.states_writer.writeheader()
            self.states_handle.flush()

    def _init_events_csv(self):
        """Create events.csv and write header."""
        # Check if file exists to avoid rewriting headers
        file_exists = self.events_file.exists()

        self.events_handle = open(self.events_file, "a", newline="")
        self.events_writer = csv.DictWriter(
            self.events_handle, fieldnames=[f.name for f in EventRow.__dataclass_fields__.values()]
        )

        if not file_exists:
            self.events_writer.writeheader()
            self.events_handle.flush()

    def log_state(self, row: StateRow):
        """
        Write a state snapshot to states.csv with periodic flushing.

        Args:
            row: StateRow dataclass instance
        """
        if self.states_writer is None or self.states_handle.closed:
            return

        self.states_writer.writerow(asdict(row))
        self.state_count += 1

        # Flush periodically to avoid data loss
        if self.state_count % self.flush_interval == 0:
            self.states_handle.flush()

    def log_event(self, row: EventRow):
        """
        Write an event to events.csv (immediately flushed).

        Args:
            row: EventRow dataclass instance
        """
        if self.events_writer is None or self.events_handle.closed:
            return

        self.events_writer.writerow(asdict(row))
        self.events_handle.flush()

    def register_drone(self):
        """
        Register a drone as using this logger.
        Increments the active drone counter.
        """
        SwarmLogger._active_drones += 1

    def unregister_drone(self):
        """
        Unregister a drone when it finishes.
        Decrements the active drone counter and closes files when all drones are done.
        """
        SwarmLogger._active_drones -= 1
        if SwarmLogger._active_drones <= 0:
            self.close()

    def now(self) -> float:
        """
        Get current Unix timestamp.

        Returns:
            float: Current time in seconds since epoch
        """
        return time.time()

    def close(self):
        """
        Flush and close all file handles.
        Safe to call multiple times (idempotent).
        """
        if self.states_handle and not self.states_handle.closed:
            self.states_handle.flush()
            self.states_handle.close()
            self.states_handle = None

        if self.events_handle and not self.events_handle.closed:
            self.events_handle.flush()
            self.events_handle.close()
            self.events_handle = None

        print(f"[Logger] Closed. Run saved to: {self.run_folder}")

    def __repr__(self):
        return f"SwarmLogger(run_folder={self.run_folder})"
