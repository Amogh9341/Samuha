import asyncio
import subprocess
import random
from mavsdk import System
from mavsdk.offboard import VelocityNedYaw

from .config import (
    MAVSDK_SERVER_BIN,
    shared_swarm_telemetry,
    TAKEOFF_ALTITUDE,
    UPDATE_RATE_HZ,
    MAX_HEIGHT_M,

    ATTRACT_KP_XY,
    ATTRACT_KI_XY,
    ATTRACT_KD_XY,

    ATTRACT_KP_Z,
    ATTRACT_KI_Z,
    ATTRACT_KD_Z,
)
from .utils import AsyncKalmanNED, calculate_swarm_velocity


class Drone:
    def __init__(self, port, target_ned):
        self.port = port
        self.target_ned = target_ned
        self.sdk_port = port + 40000
        self.drone = System(mavsdk_server_address="localhost", port=self.sdk_port)
        self.server_proc = None

        self.kf = AsyncKalmanNED()
        self.burst = 0
        # Attraction PID memory
        self.att_int_x = 0.0
        self.att_int_y = 0.0
        self.att_int_z = 0.0

        self.prev_err_x = 0.0
        self.prev_err_y = 0.0
        self.prev_err_z = 0.0

        # Motion tuning
        self.xy_weight = 1.0
        self.z_weight = 0.12
        self.max_vertical_speed = 0.35
        self.alt_hold_gain = 0.55

    async def run(self):
        self.server_proc = subprocess.Popen(
            [MAVSDK_SERVER_BIN, f"udp://:{self.port}", "-p", str(self.sdk_port)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        tele_task = None

        try:
            await self.drone.connect()
            print(f"[{self.port}] Connected")

            async def update_telemetry():
                async for pos in self.drone.telemetry.position_velocity_ned():

                    # weak link simulation
                    if random.random() < 0.10:
                        continue

                    if self.burst > 0:
                        self.burst -= 1
                        continue

                    if random.random() < 0.03:
                        self.burst = random.randint(8, 25)
                        continue

                    if random.random() < 0.12:
                        await asyncio.sleep(random.uniform(0.1, 0.6))

                    raw = [
                        pos.position.north_m,
                        pos.position.east_m,
                        pos.position.down_m,
                    ]

                    self.kf.update(raw)

                    est = self.kf.predict()
                    if est:
                        shared_swarm_telemetry[self.port] = est

            tele_task = asyncio.create_task(update_telemetry())

            async for health in self.drone.telemetry.health():
                if health.is_global_position_ok and health.is_home_position_ok:
                    break
                await asyncio.sleep(1)

            try:
                await self.drone.action.hold()
                await asyncio.sleep(1)
            except Exception:
                pass

            async for armed in self.drone.telemetry.armed():
                if not armed:
                    await self.drone.action.arm()
                break

            await self.drone.action.set_takeoff_altitude(TAKEOFF_ALTITUDE)
            await self.drone.action.takeoff()
            await asyncio.sleep(8)

            await self.drone.offboard.set_velocity_ned(
                VelocityNedYaw(0, 0, 0, 0)
            )
            await self.drone.offboard.start()

            while True:
                pred = self.kf.predict()

                if pred is None:
                    await asyncio.sleep(0.1)
                    continue

                shared_swarm_telemetry[self.port] = pred

                age = self.kf.get_age()

                max_speed = 3.0
                if age > 1.5:
                    max_speed = 1.5

                if age > 3.0:
                    await self.drone.offboard.set_velocity_ned(
                        VelocityNedYaw(0, 0, 0, 0)
                    )
                    await asyncio.sleep(0.2)
                    continue

                # Base swarm vector
                # ============================

                vn, ve, vd, dist = calculate_swarm_velocity(
                    self.port,
                    pred,
                    self.target_ned,
                    shared_swarm_telemetry,
                    max_speed=max_speed,
                    dt=1.0 / UPDATE_RATE_HZ
                )

                # ----------------------------------
                # Distance remaining to final target
                # ----------------------------------
                dx = self.target_ned[0] - pred[0]   # North/X
                dy = self.target_ned[1] - pred[1]   # East/Y
                dz = self.target_ned[2] - pred[2]   # Down/Z (NED)

                xy_dist = (dx**2 + dy**2) ** 0.5
                xyz_dist = (dx**2 + dy**2 + dz**2) ** 0.5

                print(
                    f"[{self.port}] "
                    f"Remain -> "
                    f"X:{dx:+.2f} m | "
                    f"Y:{dy:+.2f} m | "
                    f"Z:{dz:+.2f} m | "
                    f"XY:{xy_dist:.2f} m | "
                    f"3D:{xyz_dist:.2f} m"
                )
                # ----------------------------------
                # Integral accumulation
                # ----------------------------------
                self.att_int_x += dx * dt
                self.att_int_y += dy * dt
                self.att_int_z += dz * dt

                # anti-windup
                lim = 8.0

                self.att_int_x = max(-lim, min(lim, self.att_int_x))
                self.att_int_y = max(-lim, min(lim, self.att_int_y))
                self.att_int_z = max(-lim, min(lim, self.att_int_z))

                # ----------------------------------
                # Derivative
                # ----------------------------------
                ddx = (dx - self.prev_err_x) / dt
                ddy = (dy - self.prev_err_y) / dt
                ddz = (dz - self.prev_err_z) / dt

                self.prev_err_x = dx
                self.prev_err_y = dy
                self.prev_err_z = dz

                # ----------------------------------
                # Attraction PID
                # ----------------------------------
                att_vx = (
                    ATTRACT_KP_XY * dx +
                    ATTRACT_KI_XY * self.att_int_x +
                    ATTRACT_KD_XY * ddx
                )

                att_vy = (
                    ATTRACT_KP_XY * dy +
                    ATTRACT_KI_XY * self.att_int_y +
                    ATTRACT_KD_XY * ddy
                )

                att_vz = (
                    ATTRACT_KP_Z * dz +
                    ATTRACT_KI_Z * self.att_int_z +
                    ATTRACT_KD_Z * ddz
                )

                # ----------------------------------
                # Combine with repulsion vector
                # vn,ve,vd already from calculate_swarm_velocity
                # ----------------------------------
                vn += att_vx
                ve += att_vy
                vd += att_vz

                # ----------------------------------
                # Keep Z softer than XY
                # ----------------------------------
                vd *= 0.35

                # ----------------------------------
                # Height ceiling
                # ----------------------------------
                current_height = -pred[2]

                if current_height >= MAX_HEIGHT_M:
                    if vd < 0:
                        vd = 0
                    if current_height > MAX_HEIGHT_M + 0.3:
                        vd = 0.25

                # ----------------------------------
                # Clamp planar speed
                # ----------------------------------
                planar_mag = (vn**2 + ve**2) ** 0.5

                if planar_mag > max_speed:
                    scale = max_speed / planar_mag
                    vn *= scale
                    ve *= scale

                # Clamp vertical speed
                if vd > 0.5:
                    vd = 0.5
                elif vd < -0.5:
                    vd = -0.5

                await self.drone.offboard.set_velocity_ned(
                    VelocityNedYaw(vn, ve, vd, 0)
                )
                # ----------------------------------
                # Horizontal priority
                # ----------------------------------
                vn *= self.xy_weight
                ve *= self.xy_weight
                vd *= self.z_weight

                current_down = pred[2]

                # NED frame:
                # altitude above home = -down
                current_height = -current_down

                # ----------------------------------
                # HARD MAX HEIGHT RULE
                # If above max height, forbid further climb.
                # Only allow XY movement + descend.
                # ----------------------------------
                if current_height >= MAX_HEIGHT_M:

                    # if command asks to climb more, cancel it
                    if vd < 0:
                        vd = 0

                    # small forced descent to recover ceiling
                    if current_height > MAX_HEIGHT_M + 0.3:
                        vd = 0.25

                else:
                    # Normal altitude hold toward target
                    target_down = self.target_ned[2]
                    altitude_error = target_down - current_down
                    vd += self.alt_hold_gain * altitude_error

                # # Clamp vertical speed
                # if vd > self.max_vertical_speed:
                #     vd = self.max_vertical_speed
                # elif vd < -self.max_vertical_speed:
                #     vd = -self.max_vertical_speed

                # Re-limit horizontal speed
                planar_mag = (vn**2 + ve**2) ** 0.5
                if planar_mag > max_speed:
                    scale = max_speed / planar_mag
                    vn *= scale
                    ve *= scale

                await self.drone.offboard.set_velocity_ned(
                    VelocityNedYaw(vn, ve, vd, 0)
                )

                if dist < 1.0:
                    break

                await asyncio.sleep(1 / UPDATE_RATE_HZ)

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