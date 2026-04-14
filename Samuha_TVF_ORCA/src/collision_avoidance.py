# ===============================================================
# TVF + ORCA Hybrid Collision Avoidance Velocity Solver
# ===============================================================
# Combines:
#   1. TVF (Tangential Vector Field) for smooth goal attraction
#   2. TVF radial & tangential repulsion for coordinated avoidance
#   3. ORCA-style reciprocal emergency avoidance for dense crossings
#
# Better than plain TVF in congested scenarios.
# ===============================================================

import math
from .config import (
    GOAL_GAIN,
    REPULSION_GAIN,
    TANGENTIAL_GAIN,
    SAFE_RADIUS,
    INFLUENCE_RADIUS,
    MAX_SPEED_XY,
    MAX_SPEED_Z,
    DIST_EPSILON,
    VERT_VEL_SCALE,
    INFLUENCE_POWER,
    SAFE_RADIUS_SCALE,
    MIN_DIST_CLAMP,
    VERT_REPULSION_GAIN,
    TANGENT_SCALE_DIV,
    EMERGENCY_PUSH_MULT,
    TANGENTIAL_PASS_MULT,
    URGENCY_AGGRESSION_REDUCE,
)


def hybrid_velocity(my_pos, target, neighbors):
    """
    Compute desired velocity using TVF + ORCA hybrid method.

    Args:
        my_pos (list): Current position [N, E, D] in meters
        target (list): Target position [N, E, D] in meters
        neighbors (list): List of neighbor positions [[N, E, D], ...]

    Returns:
        tuple: (vn, ve, vd) desired velocity in NED frame (m/s)
    """
    px, py, pz = my_pos
    tx, ty, tz = target

    # ==========================================================
    # 1. GOAL VECTOR - Attraction toward target
    # ==========================================================
    gx = tx - px
    gy = ty - py
    gz = tz - pz

    dgoal = math.sqrt(gx * gx + gy * gy)

    if dgoal > DIST_EPSILON:
        vx = GOAL_GAIN * gx / dgoal
        vy = GOAL_GAIN * gy / dgoal
    else:
        vx = 0.0
        vy = 0.0

    vz = max(-MAX_SPEED_Z, min(MAX_SPEED_Z, VERT_VEL_SCALE * gz))

    # ==========================================================
    # 2. TVF FIELD TERMS - Smooth repulsion & tangential flow
    # ==========================================================
    for nb in neighbors:
        nx = nb[0] - px
        ny = nb[1] - py
        nz = nb[2] - pz

        dist = math.sqrt(nx * nx + ny * ny + nz * nz)

        if dist < DIST_EPSILON:
            continue

        # Within influence zone?
        if dist < INFLUENCE_RADIUS:
            # Smooth influence transition
            s = (INFLUENCE_RADIUS - dist) / INFLUENCE_RADIUS
            strength = s ** INFLUENCE_POWER + SAFE_RADIUS_SCALE * SAFE_RADIUS / max(dist, MIN_DIST_CLAMP)

            # Radial repulsion (push away)
            rx = -nx / dist
            ry = -ny / dist
            rz = -nz / dist

            vx += REPULSION_GAIN * strength * rx
            vy += REPULSION_GAIN * strength * ry
            vz += VERT_REPULSION_GAIN * strength * rz

            # Tangential side-step (circular flow)
            txv = -ry
            tyv = rx

            tangent_scale = min(1.0, dist / TANGENT_SCALE_DIV)

            # Choose circulation direction based on relative position
            sign = 1.0
            if (px + py) < (nb[0] + nb[1]):
                sign = -1.0

            vx += sign * TANGENTIAL_GAIN * tangent_scale * strength * txv
            vy += sign * TANGENTIAL_GAIN * tangent_scale * strength * tyv

    # ==========================================================
    # 3. ORCA-LIKE HARD SAFETY LAYER - Emergency reciprocal avoidance
    # ==========================================================
    for nb in neighbors:
        dx = px - nb[0]
        dy = py - nb[1]
        dz = pz - nb[2]

        dist = math.sqrt(dx * dx + dy * dy + dz * dz)

        if dist < DIST_EPSILON:
            continue

        # Emergency action when collision risk is imminent
        if dist < SAFE_RADIUS:
            nx = dx / dist
            ny = dy / dist

            urgency = (SAFE_RADIUS - dist) / SAFE_RADIUS

            # Strong lateral push away
            vx += EMERGENCY_PUSH_MULT * urgency * nx
            vy += EMERGENCY_PUSH_MULT * urgency * ny

            # Tangential pass maneuver (try to sidestep)
            txo = -ny
            tyo = nx

            vx += TANGENTIAL_PASS_MULT * urgency * txo
            vy += TANGENTIAL_PASS_MULT * urgency * tyo

            # Reduce forward aggression during close encounter
            vx *= 1.0 - URGENCY_AGGRESSION_REDUCE * urgency
            vy *= 1.0 - URGENCY_AGGRESSION_REDUCE * urgency

    # ==========================================================
    # 4. ENFORCE SPEED LIMITS
    # ==========================================================
    mag = math.sqrt(vx * vx + vy * vy)

    if mag > MAX_SPEED_XY:
        scale = MAX_SPEED_XY / mag
        vx *= scale
        vy *= scale

    vz = max(-MAX_SPEED_Z, min(MAX_SPEED_Z, vz))

    return vx, vy, vz
