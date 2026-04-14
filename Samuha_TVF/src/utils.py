import math
from .config import *
# ===============================================================
# TANGENTIAL VECTOR FIELD
# ===============================================================

def tangential_velocity(my_pos, target, neighbors):

    px, py, pz = my_pos
    tx, ty, tz = target

    # goal pull
    gx = tx - px
    gy = ty - py
    gz = tz - pz

    dgoal = math.sqrt(gx*gx + gy*gy)

    if dgoal > 1e-6:
        vx = GOAL_GAIN * gx / dgoal
        vy = GOAL_GAIN * gy / dgoal
    else:
        vx = 0.0
        vy = 0.0

    vz = max(-MAX_SPEED_Z, min(MAX_SPEED_Z, 0.45 * gz))

    # neighbor effects
    for nb in neighbors:

        nx = nb[0] - px
        ny = nb[1] - py
        nz = nb[2] - pz

        dist = math.sqrt(nx*nx + ny*ny + nz*nz)

        if dist < 1e-6:
            continue

        if dist < INFLUENCE_RADIUS:

            # smooth cubic influence
            s = (INFLUENCE_RADIUS - dist) / INFLUENCE_RADIUS
            strength = s**3 + 1.4 * (SAFE_RADIUS / max(dist, 0.25))

            # radial
            rx = -nx / dist
            ry = -ny / dist

            vx += REPULSION_GAIN * strength * rx
            vy += REPULSION_GAIN * strength * ry

            # tangential (weaker near very close range)
            txv = -ry
            tyv = rx

            tangent_scale = min(1.0, dist / 2.0)

            sign = 1.0
            if (px + py) < (nb[0] + nb[1]):
                sign = -1.0

            vx += sign * TANGENTIAL_GAIN * tangent_scale * strength * txv
            vy += sign * TANGENTIAL_GAIN * tangent_scale * strength * tyv

    # clamp XY
    mag = math.sqrt(vx*vx + vy*vy)

    if mag > MAX_SPEED_XY:
        scale = MAX_SPEED_XY / mag
        vx *= scale
        vy *= scale

    vz = max(-MAX_SPEED_Z, min(MAX_SPEED_Z, vz))

    return vx, vy, vz
