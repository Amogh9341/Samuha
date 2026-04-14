import asyncio
import subprocess
import random
import math
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw
from .filter import AsyncKalmanNED
from .utils import tangential_velocity
from .config import *

# ===============================================================
# DRONE
# ===============================================================

class Drone:
    def __init__(self, port, target_ned):
        self.port = port
        self.target = target_ned
        self.sdk_port = port + 40000

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

    async def run(self):

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

        tele_task = None

        try:
            await self.drone.connect()
            print(f"[{self.port}] Connected")

            # ---------------------------------------------------
            # TELEMETRY LOOP WITH NOISE
            # ---------------------------------------------------
            async def telemetry_loop():

                async for pos in self.drone.telemetry.position_velocity_ned():

                    # packet loss
                    if random.random() < DROP_RATE:
                        continue

                    # burst outage
                    if self.burst > 0:
                        self.burst -= 1
                        continue

                    if random.random() < BURST_START_PROB:
                        self.burst = random.randint(BURST_MIN, BURST_MAX)
                        continue

                    # delay jitter
                    if random.random() < DELAY_PROB:
                        await asyncio.sleep(
                            random.uniform(DELAY_MIN, DELAY_MAX)
                        )

                    # raw truth from SITL
                    n = pos.position.north_m
                    e = pos.position.east_m
                    d = pos.position.down_m

                    # add gaussian sensor noise
                    n += random.gauss(0.0, NOISE_STD_XY)
                    e += random.gauss(0.0, NOISE_STD_XY)
                    d += random.gauss(0.0, NOISE_STD_Z)

                    raw = [n, e, d]

                    self.kf.update(raw)

                    est = self.kf.predict()
                    if est:
                        shared_swarm_telemetry[self.port] = est

            tele_task = asyncio.create_task(telemetry_loop())

            # ---------------------------------------------------
            # HEALTH
            # ---------------------------------------------------
            async for h in self.drone.telemetry.health():
                if h.is_global_position_ok and h.is_home_position_ok:
                    print(f"[{self.port}] Health Ok")
                    break
                await asyncio.sleep(1)

            # ---------------------------------------------------
            # ARM
            # ---------------------------------------------------
            async for armed in self.drone.telemetry.armed():
                if not armed:
                    await self.drone.action.arm()
                    print(f"[{self.port}] Arming")
                break

            # ---------------------------------------------------
            # TAKEOFF
            # ---------------------------------------------------
            await self.drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE)
            await self.drone.action.takeoff()
            await asyncio.sleep(8)

            # ---------------------------------------------------
            # OFFBOARD START
            # ---------------------------------------------------
            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(0, 0, 0, 0)
            )
            await self.drone.offboard.start()

            # ---------------------------------------------------
            # MAIN LOOP
            # ---------------------------------------------------
            while True:

                my_pos = self.kf.predict()

                if my_pos is None:
                    await asyncio.sleep(0.1)
                    continue

                shared_swarm_telemetry[self.port] = my_pos

                if self.kf.age() > TELEMETRY_TIMEOUT:
                    await self.drone.offboard.set_velocity_ned(
                        VelocityNedYaw(0, 0, 0, 0)
                    )
                    await asyncio.sleep(0.2)
                    continue

                neighbors = []
                for p, val in shared_swarm_telemetry.items():
                    if p != self.port:
                        neighbors.append(val)

                vn_raw, ve_raw, vd_raw = tangential_velocity(
                    my_pos,
                    self.target,
                    neighbors
                )
   

                # -----------------------------------------
                # acceleration limiting
                # -----------------------------------------
                max_step_xy = MAX_ACCEL_XY * DT
                max_step_z  = MAX_ACCEL_Z * DT

                dv_n = vn_raw - self.cmd_vn
                dv_e = ve_raw - self.cmd_ve
                dv_d = vd_raw - self.cmd_vd

                dv_n = max(-max_step_xy, min(max_step_xy, dv_n))
                dv_e = max(-max_step_xy, min(max_step_xy, dv_e))
                dv_d = max(-max_step_z,  min(max_step_z,  dv_d))
                vn_raw = self.cmd_vn + dv_n
                ve_raw = self.cmd_ve + dv_e
                vd_raw = self.cmd_vd + dv_d

                # -----------------------------------------
                # low-pass filter
                # -----------------------------------------
                self.cmd_vn = (1-VEL_ALPHA)*self.cmd_vn + VEL_ALPHA*vn_raw
                self.cmd_ve = (1-VEL_ALPHA)*self.cmd_ve + VEL_ALPHA*ve_raw
                self.cmd_vd = (1-VEL_ALPHA)*self.cmd_vd + VEL_ALPHA*vd_raw

                # -----------------------------------------
                # deadband
                # -----------------------------------------
                if abs(self.cmd_vn) < CMD_DEADBAND:
                    self.cmd_vn = 0.0
                
                # hard ceiling
                height = -my_pos[2]
                if height >= MAX_HEIGHT_M and self.cmd_vd < 0:
                    self.cmd_vd = 0.0

                dx = self.target[0] - my_pos[0]
                dy = self.target[1] - my_pos[1]
                dz = self.target[2] - my_pos[2]

                dist = math.sqrt(dx*dx + dy*dy + dz*dz)

                print(
                    f"[{self.port}] "
                    f"X:{dx:+.2f} "
                    f"Y:{dy:+.2f} "
                    f"Z:{dz:+.2f} "
                    f"D:{dist:.2f}"
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