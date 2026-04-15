# ===============================================================
# Hardware-In-The-Loop (HITL) Drone Controller
# ===============================================================
# PX4 + MAVSDK + TVF-ORCA Hybrid with USB/Serial Communication
#
# Extends base Drone class for real hardware via USB/COM ports
# instead of simulated SITL network connections.
# ===============================================================

import asyncio
import logging
import math
from datetime import datetime
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw

from .config import *
from .kalman import AsyncKalmanNED
from .collision_avoidance import hybrid_velocity
from .logger import SwarmLogger, StateRow, EventRow


# ===============================================================
# SHARED SWARM TELEMETRY FOR HITL
# ===============================================================
shared_swarm_telemetry = {}

# ===============================================================
# HITL DRONE CONFIGURATION
# ===============================================================

class HITLConfig:
    """Configuration management for Hardware-In-The-Loop drones."""
    
    @staticmethod
    def parse_connection_string(connection_string):
        """
        Parse MAVSDK serial connection string to extract port and baud rate.
        
        Supported formats:
        - Windows: "COM3:115200"
        - Linux: "/dev/ttyUSB0:115200"
        - With prefix: "serial://COM3:115200" (prefix stripped)
        
        Args:
            connection_string (str): Connection string with port and baud rate
        
        Returns:
            tuple: (port_path, baud_rate)
        """
        # Remove serial:// prefix if present
        conn_str = connection_string.replace("serial://", "")
        
        # Split on last colon to separate port from baud rate
        parts = conn_str.rsplit(":", 1)
        
        if len(parts) == 2:
            port_path = parts[0]
            try:
                baud_rate = int(parts[1])
            except ValueError:
                baud_rate = 115200  # Default if baud rate is invalid
        else:
            port_path = parts[0]
            baud_rate = 115200  # Default baud rate
        
        return port_path, baud_rate


# ===============================================================
# HITL DRONE CLASS
# ===============================================================


class HITLDrone:
    """
    Hardware-In-The-Loop drone controller.
    
    Direct MAVSDK connection to real hardware via USB/serial.
    Uses serial connection string only, no SDK port overhead.
    
    Features:
    - USB/COM port serial communication
    - Real telemetry from PX4 flight controller
    - Direct MAVSDK connection (no server process)
    - Minimal configuration - just connection string and drone ID
    """

    def __init__(self, connection_string, drone_id, target_ned):
        """
        Initialize HITL drone with minimal configuration.
        
        Args:
            connection_string (str): Serial connection string
                Format: "COM3:115200" or "/dev/ttyUSB0:115200"
                Can optionally include "serial://" prefix
                Examples:
                  - "COM3:115200" (Windows)
                  - "COM4:57600" (Windows, custom baud)
                  - "/dev/ttyUSB0:115200" (Linux)
                  - "serial://COM3:115200" (with prefix)
            
            drone_id (int): Drone identifier (1, 2, 3, etc.)
                Used for logging and internal identification
            
            target_ned (list): Target position [N, E, D] in meters
                Required - must be provided for each drone
                Examples: [10, 2, -2], [15.5, 8.3, -2.5], etc.
        """
        # Parse connection string to extract port and baud rate
        self.port_path, self.baud_rate = HITLConfig.parse_connection_string(
            connection_string
        )
        self.drone_id = drone_id
        self.connection_string = f"serial://{self.port_path}:{self.baud_rate}"
        
        # Validate target position
        if not isinstance(target_ned, (list, tuple)) or len(target_ned) != 3:
            raise ValueError(
                f"target_ned must be a list/tuple of 3 values [N, E, D], "
                f"got {target_ned}"
            )
        
        # Setup logging
        self.logger_obj = logging.getLogger(__name__)
        self.logger_obj.info(
            f"HITL Drone {drone_id} initialized: "
            f"{self.port_path} @ {self.baud_rate} baud -> Target: {target_ned}"
        )
        
        # Store target for HITL operations
        self.target = target_ned
        
        # Initialize Kalman filter for state estimation
        self.kf = AsyncKalmanNED()
        
        # Command velocity state
        self.cmd_vn = 0.0
        self.cmd_ve = 0.0
        self.cmd_vd = 0.0
        
        # Logger (shared by all HITL drones in swarm)
        self.logger = None

    async def run(self):
        """
        Run HITL drone with full flight control.
        
        Sequence:
        1. Connect via serial to real hardware
        2. Initialize telemetry loop (no simulation - real sensor data)
        3. Arm and takeoff
        4. Enter offboard mode with collision avoidance
        5. Navigate to goal
        6. Land and cleanup
        """
        self.logger_obj.info(
            f"HITL Drone {self.drone_id}: Starting flight control via {self.connection_string}"
        )
        
        # Create MAVSDK System for direct connection
        system = System()
        tele_task = None
        
        try:
            # Initialize logger (only first drone creates it; others check)
            if not hasattr(HITLDrone, '_shared_logger'):
                run_name = f"hitl_swarm_run_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                HITLDrone._shared_logger = SwarmLogger(log_dir="logs", run_name=run_name)
            
            self.logger = HITLDrone._shared_logger
            self.logger.register_drone()
            
            # ======= CONNECT TO AUTOPILOT =======
            await system.connect(system_address=self.connection_string)
            print(f"[{self.drone_id}] Connected to HITL vehicle")
            
            self.logger.log_event(EventRow(
                timestamp=self.logger.now(),
                drone_id=self.drone_id,
                event_type="connection",
                message="Connected to MAVSDK"
            ))
            
            # ======= TELEMETRY LOOP (NO SIMULATION - REAL DATA ONLY) =======
            async def telemetry_loop():
                """Receive telemetry from real HITL vehicle (no simulated noise)."""
                async for pos in system.telemetry.position_velocity_ned():
                    # Raw reading from HITL autopilot (already real data, no noise added)
                    n = pos.position.north_m
                    e = pos.position.east_m
                    d = pos.position.down_m
                    
                    raw = [n, e, d]
                    
                    # Update Kalman filter
                    self.kf.update(raw)
                    
                    # Store for swarm coordination
                    est = self.kf.predict()
                    if est:
                        shared_swarm_telemetry[self.drone_id] = est
            
            tele_task = asyncio.create_task(telemetry_loop())
            
            # ======= WAIT FOR HEALTH OK =======
            async for h in system.telemetry.health():
                if h.is_global_position_ok and h.is_home_position_ok:
                    break
                await asyncio.sleep(HEALTH_CHECK_SLEEP)
            
            # ======= ARM =======
            async for armed in system.telemetry.armed():
                if not armed:
                    await system.action.arm()
                    self.logger.log_event(EventRow(
                        timestamp=self.logger.now(),
                        drone_id=self.drone_id,
                        event_type="arm",
                        message="Armed and ready"
                    ))
                break
            
            # ======= TAKEOFF =======
            await system.action.set_takeoff_altitude(TAKEOFF_ALTITUDE)
            await system.action.takeoff()
            await asyncio.sleep(TAKEOFF_WAIT_TIME)
            
            self.logger.log_event(EventRow(
                timestamp=self.logger.now(),
                drone_id=self.drone_id,
                event_type="takeoff",
                message=f"Took off to {TAKEOFF_ALTITUDE}m"
            ))
            
            # ======= OFFBOARD MODE START =======
            await system.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, 0))
            await system.offboard.start()
            
            self.logger.log_event(EventRow(
                timestamp=self.logger.now(),
                drone_id=self.drone_id,
                event_type="offboard",
                message="Entered offboard mode"
            ))
            
            # ======= MAIN CONTROL LOOP WITH COLLISION AVOIDANCE =======
            while True:
                my_pos = self.kf.predict()
                
                if my_pos is None:
                    await asyncio.sleep(PREDICT_SLEEP)
                    continue
                
                shared_swarm_telemetry[self.drone_id] = my_pos
                
                # Timeout protection: no telemetry -> stop
                if self.kf.age() > TELEMETRY_TIMEOUT:
                    self.logger.log_event(EventRow(
                        timestamp=self.logger.now(),
                        drone_id=self.drone_id,
                        event_type="telemetry_timeout",
                        message=f"No telemetry for {self.kf.age():.2f}s"
                    ))
                    await system.offboard.set_velocity_ned(
                        VelocityNedYaw(0, 0, 0, 0)
                    )
                    await asyncio.sleep(TELEMETRY_STOP_SLEEP)
                    continue
                
                # Gather neighbors from swarm
                neighbors = []
                for drone_id, val in shared_swarm_telemetry.items():
                    if drone_id != self.drone_id:
                        neighbors.append(val)
                
                # Compute desired velocity using TVF-ORCA hybrid
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
                # Distance to goal
                # -----------------------------------------
                dx = self.target[0] - my_pos[0]
                dy = self.target[1] - my_pos[1]
                dz = self.target[2] - my_pos[2]
                
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                
                print(
                    f"[{self.drone_id}] "
                    f"X:{dx:+.2f} "
                    f"Y:{dy:+.2f} "
                    f"Z:{dz:+.2f} "
                    f"D:{dist:.2f}"
                )
                
                # Send velocity command
                await system.offboard.set_velocity_ned(
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
                    drone_id=self.drone_id,
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
                    burst_flag=0  # No simulation in HITL
                ))
                
                # Check if goal reached
                if dist < GOAL_TOLERANCE:
                    break
                
                await asyncio.sleep(DT)
            
            # ======= LANDING =======
            self.logger.log_event(EventRow(
                timestamp=self.logger.now(),
                drone_id=self.drone_id,
                event_type="landing",
                message="Reached goal, landing..."
            ))
            await system.action.land()
        
        except Exception as e:
            self.logger_obj.error(f"HITL Drone {self.drone_id} error: {e}")
            raise
        
        finally:
            # Cleanup
            if tele_task:
                tele_task.cancel()
                try:
                    await tele_task
                except:
                    pass
            
            # Close connection
            await system.close()
            self.logger_obj.info(f"HITL Drone {self.drone_id}: Disconnected")
            
            # Unregister drone from logger (closes when all drones are done)
            if self.logger:
                self.logger.unregister_drone()

    def get_status(self):
        """
        Get HITL drone status information.
        
        Returns:
            dict: Status dictionary with drone info
        """
        return {
            "drone_id": self.drone_id,
            "port_path": self.port_path,
            "baud_rate": self.baud_rate,
            "connection_string": self.connection_string,
            "target_ned": self.target,
            "is_hitl": True,
        }

    async def test_telemetry(self, stream_type="position", duration=None):
        """
        Test telemetry reception from HITL drone via serial connection.
        
        Connects directly to the drone and streams selected telemetry.
        Useful for debugging and verifying drone is connected and sending data.
        
        Args:
            stream_type (str): Type of telemetry to stream
                Options: "position", "attitude", "battery", "all"
                Default: "position"
            
            duration (float): Duration in seconds to stream telemetry
                If None, streams indefinitely until KeyboardInterrupt
        
        Examples:
            # Stream position for 30 seconds
            await drone.test_telemetry(stream_type="position", duration=30)
            
            # Stream all telemetry indefinitely
            await drone.test_telemetry(stream_type="all")
        """
        self.logger_obj.info(
            f"HITL Drone {self.drone_id}: Starting telemetry test ({stream_type}) via {self.connection_string}"
        )
        
        # Create direct MAVSDK connection to serial port (no server process needed)
        system = System()
        
        try:
            # Connect directly to autopilot via serial
            await system.connect(system_address=self.connection_string)
            print(f"[Drone {self.drone_id}] Connected to HITL vehicle via {self.connection_string}")
            
            # Wait for connection to stabilize
            async for state in system.core.connection_state():
                if state.is_connected:
                    print(f"[Drone {self.drone_id}] Ready for telemetry streaming")
                    break
            
            # Stream telemetry based on request
            start_time = asyncio.get_event_loop().time()
            
            if stream_type.lower() == "position":
                async for pos in system.telemetry.position():
                    print(
                        f"[{self.drone_id}] POS: lat={pos.latitude_deg:.6f} "
                        f"lon={pos.longitude_deg:.6f} rel_alt={pos.relative_altitude_m:.2f}m"
                    )
                    if duration and (asyncio.get_event_loop().time() - start_time) > duration:
                        break
            
            elif stream_type.lower() == "attitude":
                async for att in system.telemetry.attitude_euler():
                    print(
                        f"[{self.drone_id}] ATT: roll={att.roll_deg:.2f}° "
                        f"pitch={att.pitch_deg:.2f}° yaw={att.yaw_deg:.2f}°"
                    )
                    if duration and (asyncio.get_event_loop().time() - start_time) > duration:
                        break
            
            elif stream_type.lower() == "battery":
                async for batt in system.telemetry.battery():
                    remaining = batt.remaining_percent if batt.remaining_percent is not None else 0.0
                    print(f"[{self.drone_id}] BAT: {(remaining*100):.1f}% voltage={batt.voltage_v:.2f}V")
                    if duration and (asyncio.get_event_loop().time() - start_time) > duration:
                        break
            
            elif stream_type.lower() == "all":
                # Stream all three simultaneously
                async def pos_stream():
                    async for pos in system.telemetry.position():
                        print(
                            f"[{self.drone_id}] POS: lat={pos.latitude_deg:.6f} "
                            f"lon={pos.longitude_deg:.6f} rel_alt={pos.relative_altitude_m:.2f}m"
                        )
                
                async def att_stream():
                    async for att in system.telemetry.attitude_euler():
                        print(
                            f"[{self.drone_id}] ATT: roll={att.roll_deg:.2f}° "
                            f"pitch={att.pitch_deg:.2f}° yaw={att.yaw_deg:.2f}°"
                        )
                
                async def bat_stream():
                    async for batt in system.telemetry.battery():
                        remaining = batt.remaining_percent if batt.remaining_percent is not None else 0.0
                        print(f"[{self.drone_id}] BAT: {(remaining*100):.1f}% voltage={batt.voltage_v:.2f}V")
                
                # Run all streams concurrently until duration expires
                pos_task = asyncio.create_task(pos_stream())
                att_task = asyncio.create_task(att_stream())
                bat_task = asyncio.create_task(bat_stream())
                
                if duration:
                    await asyncio.sleep(duration)
                    pos_task.cancel()
                    att_task.cancel()
                    bat_task.cancel()
                else:
                    # Wait indefinitely
                    await asyncio.gather(pos_task, att_task, bat_task)
            
            else:
                print(f"Unsupported stream type: {stream_type}")
                print("Supported: position, attitude, battery, all")
        
        except Exception as e:
            self.logger_obj.error(f"HITL Drone {self.drone_id} telemetry test error: {e}")
            raise
        
        finally:
            # Disconnect
            await system.close()
            self.logger_obj.info(f"HITL Drone {self.drone_id}: Telemetry test disconnected")

