import time
import math
import numpy as np

class AsyncKalmanNED:
    def __init__(self):
        self.x = np.zeros(6)  # [n,e,d,vn,ve,vd]
        self.P = np.eye(6) * 50.0
        self.R = np.eye(3) * 0.6
        self.initialized = False
        self.last_time = time.time()

    def _F(self, dt):
        F = np.eye(6)
        for i in range(3):
            F[i, i+3] = dt
        return F

    def _Q(self, dt, accel_var=0.8):
        Q = np.zeros((6,6))
        for i in range(3):
            Q[i,i] = 0.25 * dt**4 * accel_var
            Q[i,i+3] = 0.5 * dt**3 * accel_var
            Q[i+3,i] = 0.5 * dt**3 * accel_var
            Q[i+3,i+3] = dt**2 * accel_var
        return Q

    def predict(self):
        if not self.initialized:
            return None
        now = time.time()
        dt = max(0.01, now - self.last_time)
        self.last_time = now
        F = self._F(dt)
        Q = self._Q(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        return self.x[:3].tolist()

    def update(self, measurement):
        z = np.array(measurement, dtype=float)
        now = time.time()
        if not self.initialized:
            self.x[:3] = z
            self.last_time = now
            self.initialized = True
            return
        dt = max(0.01, now - self.last_time)
        self.last_time = now
        F = self._F(dt)
        Q = self._Q(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        H = np.zeros((3,6))
        H[:3,:3] = np.eye(3)
        y = z - H @ self.x
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(6) - K @ H) @ self.P

    def get_age(self):
        return time.time() - self.last_time


_swarm_pid_state = {}


def calculate_swarm_velocity(
    port,
    my_pos,
    target_ned,
    shared_swarm_telemetry,
    max_speed=3.0,
    repel_radius=8.0,
    dt=0.05,
):
    from .config import (
        REPULSION_KP,
        REPULSION_KI,
        REPULSION_KD,
    )

    mn, me, md = my_pos
    tn, te, td = target_ned

    # -------------------------------------------
    # attraction to target
    # -------------------------------------------
    att_n = tn - mn
    att_e = te - me
    att_d = td - md

    dist = math.sqrt(att_n**2 + att_e**2 + att_d**2)

    if dist > 1e-6:
        att_n /= dist
        att_e /= dist
        att_d /= dist

    # -------------------------------------------
    # raw repulsion
    # -------------------------------------------
    raw_rep_n = 0.0
    raw_rep_e = 0.0
    raw_rep_d = 0.0

    for other_port, pos in shared_swarm_telemetry.items():

        if other_port == port:
            continue

        on, oe, od = pos

        dn = mn - on
        de = me - oe
        dd = md - od

        d = math.sqrt(dn**2 + de**2 + dd**2)

        if 0.001 < d < repel_radius:

            strength = (repel_radius - d) / repel_radius

            raw_rep_n += strength * dn / d
            raw_rep_e += strength * de / d
            raw_rep_d += strength * dd / d

    # -------------------------------------------
    # initialize PID memory
    # -------------------------------------------
    if port not in _swarm_pid_state:
        _swarm_pid_state[port] = {
            "int_n": 0.0,
            "int_e": 0.0,
            "int_d": 0.0,
            "prev_n": 0.0,
            "prev_e": 0.0,
            "prev_d": 0.0,
        }

    s = _swarm_pid_state[port]

    # -------------------------------------------
    # integral
    # -------------------------------------------
    s["int_n"] += raw_rep_n * dt
    s["int_e"] += raw_rep_e * dt
    s["int_d"] += raw_rep_d * dt

    lim = 5.0

    s["int_n"] = max(-lim, min(lim, s["int_n"]))
    s["int_e"] = max(-lim, min(lim, s["int_e"]))
    s["int_d"] = max(-lim, min(lim, s["int_d"]))

    # -------------------------------------------
    # derivative
    # -------------------------------------------
    der_n = (raw_rep_n - s["prev_n"]) / dt
    der_e = (raw_rep_e - s["prev_e"]) / dt
    der_d = (raw_rep_d - s["prev_d"]) / dt

    s["prev_n"] = raw_rep_n
    s["prev_e"] = raw_rep_e
    s["prev_d"] = raw_rep_d

    # -------------------------------------------
    # PID repulsion output
    # -------------------------------------------
    rep_n = (
        REPULSION_KP * raw_rep_n +
        REPULSION_KI * s["int_n"] +
        REPULSION_KD * der_n
    )

    rep_e = (
        REPULSION_KP * raw_rep_e +
        REPULSION_KI * s["int_e"] +
        REPULSION_KD * der_e
    )

    rep_d = (
        REPULSION_KP * raw_rep_d +
        REPULSION_KI * s["int_d"] +
        REPULSION_KD * der_d
    )

    # -------------------------------------------
    # keep Z weak
    # -------------------------------------------
    rep_d *= 0.10
    att_d *= 0.10

    # -------------------------------------------
    # final command
    # -------------------------------------------
    vn = att_n + rep_n
    ve = att_e + rep_e
    vd = att_d + rep_d

    mag = math.sqrt(vn**2 + ve**2 + vd**2)

    if mag > max_speed:
        scale = max_speed / mag
        vn *= scale
        ve *= scale
        vd *= scale

    return vn, ve, vd, dist