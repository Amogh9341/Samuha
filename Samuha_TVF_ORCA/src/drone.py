# ===============================================================
# Drone Controller with PX4 + MAVSDK + TVF-ORCA Hybrid
# ===============================================================

import asyncio
import subprocess
import math
import random
import time
from datetime import datetime
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw

from .config import *
from .kalman import AsyncKalmanNED
from .collision_avoidance import hybrid_velocity
from .logger import SwarmLogger, StateRow, EventRow


# ===============================================================
# SHARED SWARM TELEMETRY
# ===============================================================

shared_swarm_telemetry = {}


# ===============================================================
# DRONE CLASS
# ===============================================================


class Drone:
    """
    Single drone controller interfacing with PX4 via MAVSDK.
    
    Features:
    - Telemetry with simulated impairments (noise, drops, jitter)
    - Kalman filtering for position/velocity estimation
    - TVF-ORCA hybrid collision avoidance
    - Acceleration limiting and velocity filtering
    """

    def __init__(self, port, target_ned):
        """
        Initialize drone with telemetry port and target.

        Args:
            port (int): Telemetry port (e.g., 14540)
            target_ned (list): Target position [N, E, D] in meters
        """
        self.port = port
        self.target = target_ned
        self.sdk_port = port + SDK_PORT_OFFSET

        self.drone = System(
            mavsdk_server_address="localhost", port=self.sdk_port
        )

        self.server_proc = None
        self.kf = AsyncKalmanNED()
        self.burst = 0  # Burst outage counter
        self.cmd_vn = 0.0  # Commanded velocity North
        self.cmd_ve = 0.0  # Commanded velocity East
        self.cmd_vd = 0.0  # Commanded velocity Down

        # Logger initialization (shared by all drones in swarm)
        self.logger = None

    async def run(self):
        """
        Main async run loop for the drone.
        
        Sequence:
        1. Start MAVSDK server
        2. Connect to autopilot
        3. Arm and takeoff
        4. Enter offboard mode with collision avoidance
        5. Navigate to goal
        6. Land
        """

        self.server_proc = subprocess.Popen(
            [
                MAVSDK_SERVER_BIN,
                f"udp://:{self.port}",
                "-p",
                str(self.sdk_port),
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        # Initialize logger (only first drone creates it; others check)
        if not hasattr(Drone, '_shared_logger'):
            run_name = f"swarm_run_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            Drone._shared_logger = SwarmLogger(log_dir="logs", run_name=run_name)

        self.logger = Drone._shared_logger
        self.logger.register_drone()  # Register this drone with the logger

        tele_task = None

        try:
            await self.drone.connect()
            print(f"[{self.port}] Connected")

            self.logger.log_event(EventRow(
                timestamp=self.logger.now(),
                drone_id=self.port,
                event_type="connection",
                message="Connected to MAVSDK"
            ))

            # ---------------------------------------------------
            # TELEMETRY LOOP WITH NOISE SIMULATION
            # ---------------------------------------------------
            async def telemetry_loop():
                """Receive telemetry with simulated impairments."""
                async for pos in self.drone.telemetry.position_velocity_ned():
                    # Packet loss
                    if random.random() < DROP_RATE:
                        continue

                    # Burst outage (e.g., radio interference)
                    if self.burst > 0:
                        self.burst -= 1
                        continue

                    if random.random() < BURST_START_PROB:
                        self.burst = random.randint(BURST_MIN, BURST_MAX)
                        continue

                    # Delay jitter
                    if random.random() < DELAY_PROB:
                        await asyncio.sleep(
                            random.uniform(DELAY_MIN, DELAY_MAX)
                        )

                    # Raw truth from SITL
                    n = pos.position.north_m
                    e = pos.position.east_m
                    d = pos.position.down_m

                    # Add Gaussian sensor noise
                    n += random.gauss(0.0, NOISE_STD_XY)
                    e += random.gauss(0.0, NOISE_STD_XY)
                    d += random.gauss(0.0, NOISE_STD_Z)

                    raw = [n, e, d]

                    # Update Kalman filter
                    self.kf.update(raw)

                    # Store for other drones
                    est = self.kf.predict()
                    if est:
                        shared_swarm_telemetry[self.port] = est

            tele_task = asyncio.create_task(telemetry_loop())

            # ---------------------------------------------------
            # WAIT FOR HEALTH OK
            # ---------------------------------------------------
            async for h in self.drone.telemetry.health():
                if h.is_global_position_ok and h.is_home_position_ok:
                    break
                await asyncio.sleep(HEALTH_CHECK_SLEEP)

            # ---------------------------------------------------
            # ARM
            # ---------------------------------------------------
            async for armed in self.drone.telemetry.armed():
                if not armed:
                    await self.drone.action.arm()
                    self.logger.log_event(EventRow(
                        timestamp=self.logger.now(),
                        drone_id=self.port,
                        event_type="arm",
                        message="Armed and ready"
                    ))
                break

            # ---------------------------------------------------
            # TAKEOFF
            # ---------------------------------------------------
            await self.drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE)
            await self.drone.action.takeoff()
            await asyncio.sleep(TAKEOFF_WAIT_TIME)

            self.logger.log_event(EventRow(
                timestamp=self.logger.now(),
                drone_id=self.port,
                event_type="takeoff",
                message=f"Took off to {TAKEOFF_ALTITUDE}m"
            ))

            # ---------------------------------------------------
            # OFFBOARD MODE START
            # ---------------------------------------------------
            await self.drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
            await self.drone.offboard.start()

            self.logger.log_event(EventRow(
                timestamp=self.logger.now(),
                drone_id=self.port,
                event_type="offboard",
                message="Entered offboard mode"
            ))

            # ---------------------------------------------------
            # MAIN CONTROL LOOP
            # ---------------------------------------------------
            while True:
                my_pos = self.kf.predict()

                if my_pos is None:
                    await asyncio.sleep(PREDICT_SLEEP)
                    continue

                shared_swarm_telemetry[self.port] = my_pos

                # Timeout protection: no telemetry -> stop
                if self.kf.age() > TELEMETRY_TIMEOUT:
                    self.logger.log_event(EventRow(
                        timestamp=self.logger.now(),
                        drone_id=self.port,
                        event_type="telemetry_timeout",
                        message=f"No telemetry for {self.kf.age():.2f}s"
                    ))
                    await self.drone.offboard.set_velocity_ned(
                        VelocityNedYaw(0, 0, 0, 0)
                    )
                    await asyncio.sleep(TELEMETRY_STOP_SLEEP)
                    continue

                # Gather neighbors from swarm
                neighbors = []
                for p, val in shared_swarm_telemetry.items():
                    if p != self.port:
                        neighbors.append(val)

                # Compute desired velocity
                vn_raw, ve_raw, vd_raw = hybrid_velocity(
                    my_pos, self.target, neighbors
                )

                # Store raw velocity for logging (before filtering/limiting)
                raw_vn_raw = vn_raw
                raw_ve_raw = ve_raw
                raw_vd_raw = vd_raw

                # -----------------------------------------
                # Acceleration Limiting
                # -----------------------------------------
                max_step_xy = MAX_ACCEL_XY * DT
                max_step_z = MAX_ACCEL_Z * DT

                dv_n = vn_raw - self.cmd_vn
                dv_e = ve_raw - self.cmd_ve
                dv_d = vd_raw - self.cmd_vd

                dv_n = max(-max_step_xy, min(max_step_xy, dv_n))
                dv_e = max(-max_step_xy, min(max_step_xy, dv_e))
                dv_d = max(-max_step_z, min(max_step_z, dv_d))

                vn_raw = self.cmd_vn + dv_n
                ve_raw = self.cmd_ve + dv_e
                vd_raw = self.cmd_vd + dv_d

                # -----------------------------------------
                # Low-Pass Filter for Smooth Commands
                # -----------------------------------------
                self.cmd_vn = (1 - VEL_ALPHA) * self.cmd_vn + VEL_ALPHA * vn_raw
                self.cmd_ve = (1 - VEL_ALPHA) * self.cmd_ve + VEL_ALPHA * ve_raw
                self.cmd_vd = (1 - VEL_ALPHA) * self.cmd_vd + VEL_ALPHA * vd_raw

                # -----------------------------------------
                # Deadband
                # -----------------------------------------
                if abs(self.cmd_vn) < CMD_DEADBAND:
                    self.cmd_vn = 0.0
                if abs(self.cmd_ve) < CMD_DEADBAND:
                    self.cmd_ve = 0.0

                # -----------------------------------------
                # Hard Ceiling Constraint
                # -----------------------------------------
                height = -my_pos[2]
                if height >= MAX_HEIGHT_M and self.cmd_vd < 0:
                    self.cmd_vd = 0.0

                # -----------------------------------------
                # DEBUG: Distance to goal
                # -----------------------------------------
                dx = self.target[0] - my_pos[0]
                dy = self.target[1] - my_pos[1]
                dz = self.target[2] - my_pos[2]

                dist = math.sqrt(dx * dx + dy * dy + dz * dz)

                print(
                    f"[{self.port}] "
                    f"X:{dx:+.2f} "
                    f"Y:{dy:+.2f} "
                    f"Z:{dz:+.2f} "
                    f"D:{dist:.2f}"
                )

                # Send velocity command
                await self.drone.offboard.set_velocity_ned(
                    VelocityNedYaw(self.cmd_vn, self.cmd_ve, self.cmd_vd, 0)
                )

                # Calculate neighbor info
                nearest_neighbor = -1.0
                if neighbors:
                    dists_to_neighbors = []
                    for nb in neighbors:
                        dnx = my_pos[0] - nb[0]
                        dny = my_pos[1] - nb[1]
                        dist_n = math.sqrt(dnx * dnx + dny * dny)
                        dists_to_neighbors.append(dist_n)
                    nearest_neighbor = min(dists_to_neighbors)

                # Log state snapshot
                self.logger.log_state(StateRow(
                    timestamp=self.logger.now(),
                    drone_id=self.port,
                    north=my_pos[0],
                    east=my_pos[1],
                    down=my_pos[2],
                    cmd_vn=self.cmd_vn,
                    cmd_ve=self.cmd_ve,
                    cmd_vd=self.cmd_vd,
                    raw_vn=raw_vn_raw,
                    raw_ve=raw_ve_raw,
                    raw_vd=raw_vd_raw,
                    dx=dx,
                    dy=dy,
                    dz=dz,
                    dist_goal=dist,
                    nearest_neighbor=nearest_neighbor,
                    num_neighbors=len(neighbors),
                    telemetry_age=self.kf.age(),
                    timeout_flag=0,
                    packet_drop_flag=0,
                    burst_flag=1 if self.burst > 0 else 0
                ))

                # Check if goal reached
                if dist < GOAL_TOLERANCE:
                    break

                await asyncio.sleep(DT)

            # Land
            self.logger.log_event(EventRow(
                timestamp=self.logger.now(),
                drone_id=self.port,
                event_type="landing",
                message="Reached goal, landing..."
            ))
            await self.drone.action.land()

        finally:
            # Cleanup
            if tele_task:
                tele_task.cancel()
                try:
                    await tele_task
                except:
                    pass

            if self.server_proc:
                self.server_proc.terminate()

            # Unregister drone from logger (closes when all drones are done)
            if self.logger:
                self.logger.unregister_drone()
