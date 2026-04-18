# ===============================================================
# Drone Controller with PX4 + MAVSDK + TVF-ORCA Hybrid + ROS2
# MINIMAL PATCH VERSION
# Existing functions preserved:
#   __init__()
#   run()
#   _run_main_loop()
# Only backend telemetry sharing changed
# ===============================================================

import asyncio
import subprocess
import math
import random
import time
from datetime import datetime

from mavsdk import System
from mavsdk.offboard import VelocityNedYaw

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped

from .config import *
from .kalman import AsyncKalmanNED
from .collision_avoidance import hybrid_velocity
from .logger import SwarmLogger, StateRow, EventRow


# ===============================================================
# DRONE CLASS
# ===============================================================

class Drone(Node):
    """
    Single drone controller interfacing with PX4 via MAVSDK.

    ROS2 upgraded version:
    - Publishes own telemetry
    - Subscribes to other drones telemetry
    - Existing methods preserved
    """

    def __init__(self, port, target_ned, sitl=True):
        """
        Initialize drone with telemetry port and target.

        Args:
            port (int): Telemetry port
            target_ned (list): Target [N,E,D]
            sitl (bool): SITL mode
        """

        Node.__init__(self, f"drone_{port}")

        self.port = port
        self.target = target_ned
        self.sdk_port = port + SDK_PORT_OFFSET
        self.SITL = sitl

        self.drone = System(
            mavsdk_server_address="localhost",
            port=self.sdk_port
        )

        self.server_proc = None
        self.kf = AsyncKalmanNED()

        self.burst = 0

        self.cmd_vn = 0.0
        self.cmd_ve = 0.0
        self.cmd_vd = 0.0

        self.logger = None

        # ===================================================
        # ROS2 TELEMETRY STORAGE
        # ===================================================

        self.neighbor_positions = {}

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # own publisher
        self.telemetry_pub = self.create_publisher(
            PoseStamped,
            f"/drone_{self.port}/telemetry",
            qos
        )

        # internal callback function
        def neighbor_callback(msg, drone_port):

            self.neighbor_positions[drone_port] = [
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            ]

        # subscribe to all drones in port range
        # adjust range as needed
        for p in range(14540, 14560):

            if p == self.port:
                continue

            self.create_subscription(
                PoseStamped,
                f"/drone_{p}/telemetry",
                lambda msg, x=p: neighbor_callback(msg, x),
                qos
            )

    # ===============================================================
    # EXISTING FUNCTION KEPT
    # ===============================================================

    async def run(self):
        """
        Main async run loop for drone.
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

        await self._run_main_loop()

    # ===============================================================
    # EXISTING FUNCTION KEPT
    # ===============================================================

    async def _run_main_loop(self):

        if not hasattr(Drone, "_shared_logger"):
            run_name = f"swarm_run_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            Drone._shared_logger = SwarmLogger(
                log_dir="logs",
                run_name=run_name
            )

        self.logger = Drone._shared_logger
        self.logger.register_drone()

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

            # ===================================================
            # TELEMETRY LOOP
            # ===================================================

            async def telemetry_loop():

                async for pos in self.drone.telemetry.position_velocity_ned():

                    if self.SITL:

                        if random.random() < DROP_RATE:
                            continue

                        if self.burst > 0:
                            self.burst -= 1
                            continue

                        if random.random() < BURST_START_PROB:
                            self.burst = random.randint(
                                BURST_MIN,
                                BURST_MAX
                            )
                            continue

                        if random.random() < DELAY_PROB:
                            await asyncio.sleep(
                                random.uniform(
                                    DELAY_MIN,
                                    DELAY_MAX
                                )
                            )

                    n = pos.position.north_m
                    e = pos.position.east_m
                    d = pos.position.down_m

                    if self.SITL:
                        n += random.gauss(0.0, NOISE_STD_XY)
                        e += random.gauss(0.0, NOISE_STD_XY)
                        d += random.gauss(0.0, NOISE_STD_Z)

                    raw = [n, e, d]

                    self.kf.update(raw)

                    est = self.kf.predict()

                    if est:

                        # publish instead of shared dict
                        msg = PoseStamped()

                        msg.pose.position.x = est[0]
                        msg.pose.position.y = est[1]
                        msg.pose.position.z = est[2]

                        self.telemetry_pub.publish(msg)

            tele_task = asyncio.create_task(
                telemetry_loop()
            )

            # ===================================================
            # WAIT HEALTH
            # ===================================================

            async for h in self.drone.telemetry.health():

                if h.is_global_position_ok and h.is_home_position_ok:
                    break

                await asyncio.sleep(
                    HEALTH_CHECK_SLEEP
                )

            # ===================================================
            # ARM
            # ===================================================

            async for armed in self.drone.telemetry.armed():

                if not armed:
                    await self.drone.action.arm()

                break

            # ===================================================
            # TAKEOFF
            # ===================================================

            await self.drone.action.set_takeoff_altitude(
                TAKEOFF_ALTITUDE
            )

            await self.drone.action.takeoff()

            await asyncio.sleep(
                TAKEOFF_WAIT_TIME
            )

            # ===================================================
            # OFFBOARD START
            # ===================================================

            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(0, 0, 0, 0)
            )

            await self.drone.offboard.start()

            # ===================================================
            # MAIN CONTROL LOOP
            # ===================================================

            while True:

                rclpy.spin_once(
                    self,
                    timeout_sec=0.0
                )

                my_pos = self.kf.predict()

                if my_pos is None:
                    await asyncio.sleep(
                        PREDICT_SLEEP
                    )
                    continue

                if self.kf.age() > TELEMETRY_TIMEOUT:

                    await self.drone.offboard.set_velocity_ned(
                        VelocityNedYaw(0, 0, 0, 0)
                    )

                    await asyncio.sleep(
                        TELEMETRY_STOP_SLEEP
                    )

                    continue

                # ===========================================
                # gather neighbors from ROS2
                # ===========================================

                neighbors = list(
                    self.neighbor_positions.values()
                )

                vn_raw, ve_raw, vd_raw = hybrid_velocity(
                    my_pos,
                    self.target,
                    neighbors
                )

                raw_vn_raw = vn_raw
                raw_ve_raw = ve_raw
                raw_vd_raw = vd_raw

                # ===========================================
                # acceleration limit
                # ===========================================

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

                # ===========================================
                # smoothing
                # ===========================================

                self.cmd_vn = (
                    (1 - VEL_ALPHA) * self.cmd_vn
                    + VEL_ALPHA * vn_raw
                )

                self.cmd_ve = (
                    (1 - VEL_ALPHA) * self.cmd_ve
                    + VEL_ALPHA * ve_raw
                )

                self.cmd_vd = (
                    (1 - VEL_ALPHA) * self.cmd_vd
                    + VEL_ALPHA * vd_raw
                )

                # deadband
                if abs(self.cmd_vn) < CMD_DEADBAND:
                    self.cmd_vn = 0.0

                if abs(self.cmd_ve) < CMD_DEADBAND:
                    self.cmd_ve = 0.0

                # ceiling
                height = -my_pos[2]

                if height >= MAX_HEIGHT_M and self.cmd_vd < 0:
                    self.cmd_vd = 0.0

                dx = self.target[0] - my_pos[0]
                dy = self.target[1] - my_pos[1]
                dz = self.target[2] - my_pos[2]

                dist = math.sqrt(
                    dx * dx + dy * dy + dz * dz
                )

                print(
                    f"[{self.port}] D:{dist:.2f}"
                )

                await self.drone.offboard.set_velocity_ned(
                    VelocityNedYaw(
                        self.cmd_vn,
                        self.cmd_ve,
                        self.cmd_vd,
                        0
                    )
                )

                if dist < GOAL_TOLERANCE:
                    break

                await asyncio.sleep(DT)

            # ===================================================
            # LAND
            # ===================================================

            await self.drone.action.land()

        finally:

            if tele_task:
                tele_task.cancel()

                try:
                    await tele_task
                except:
                    pass

            if self.server_proc:
                self.server_proc.terminate()

            if self.logger:
                self.logger.unregister_drone()