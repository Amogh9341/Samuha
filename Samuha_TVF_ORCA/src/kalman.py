# ===============================================================
# Asynchronous Kalman Filter for NED Position/Velocity Estimation
# ===============================================================

import time
import numpy as np

from .config import (
    MINIMUM_DT,
    KF_VEL_NE_DECAY,
    KF_VEL_NE_UPDATE,
    KF_VEL_D_DECAY,
    KF_VEL_D_UPDATE,
)


class AsyncKalmanNED:
    """
    Simple Kalman filter for estimating position and velocity in NED frame.
    
    State vector: [N, E, D, V_N, V_E, V_D]
    - N, E, D: position in North, East, Down (meters)
    - V_N, V_E, V_D: velocity in North, East, Down (m/s)
    """

    def __init__(self):
        """Initialize the filter with zero state."""
        self.x = np.zeros(6)  # [N, E, D, VN, VE, VD]
        self.initialized = False
        self.last = time.time()

    def predict(self):
        """
        Predict next state using a constant velocity model.
        
        Returns:
            list or None: Predicted position [N, E, D] or None if not yet initialized
        """
        if not self.initialized:
            return None

        now = time.time()
        dt = max(MINIMUM_DT, now - self.last)
        self.last = now

        # Constant velocity model
        self.x[0] += self.x[3] * dt
        self.x[1] += self.x[4] * dt
        self.x[2] += self.x[5] * dt

        return self.x[:3].tolist()

    def update(self, meas):
        """
        Update filter with a new position measurement.
        
        Args:
            meas (list): [N, E, D] position measurement in meters
        """
        z = np.array(meas)

        if not self.initialized:
            self.x[:3] = z
            self.last = time.time()
            self.initialized = True
            return

        dt = max(MINIMUM_DT, time.time() - self.last)

        # Estimate velocity from position change
        vx = (z[0] - self.x[0]) / dt
        vy = (z[1] - self.x[1]) / dt
        vz = (z[2] - self.x[2]) / dt

        # Update position
        self.x[:3] = z

        # Low-pass filter velocity with different smoothing per axis
        # North and East velocities
        self.x[3] = KF_VEL_NE_DECAY * self.x[3] + KF_VEL_NE_UPDATE * vx
        self.x[4] = KF_VEL_NE_DECAY * self.x[4] + KF_VEL_NE_UPDATE * vy
        # Down velocity: smoother
        self.x[5] = KF_VEL_D_DECAY * self.x[5] + KF_VEL_D_UPDATE * vz

        self.last = time.time()

    def age(self):
        """
        Get the age of the last update.
        
        Returns:
            float: Seconds since last update
        """
        return time.time() - self.last
