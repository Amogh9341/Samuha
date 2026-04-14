import time
import numpy as np

# ===============================================================
# SIMPLE KALMAN FILTER
# ===============================================================

class AsyncKalmanNED:
    def __init__(self):
        self.x = np.zeros(6)
        self.initialized = False
        self.last = time.time()

    def predict(self):
        if not self.initialized:
            return None

        now = time.time()
        dt = max(0.01, now - self.last)
        self.last = now

        self.x[0] += self.x[3] * dt
        self.x[1] += self.x[4] * dt
        self.x[2] += self.x[5] * dt

        return self.x[:3].tolist()

    def update(self, meas):
        z = np.array(meas)

        if not self.initialized:
            self.x[:3] = z
            self.last = time.time()
            self.initialized = True
            return

        dt = max(0.01, time.time() - self.last)

        vx = (z[0] - self.x[0]) / dt
        vy = (z[1] - self.x[1]) / dt
        vz = (z[2] - self.x[2]) / dt

        self.x[:3] = z
        self.x[3] = 0.88 * self.x[3] + 0.12 * vx
        self.x[4] = 0.88 * self.x[4] + 0.12 * vy
        self.x[5] = 0.92 * self.x[5] + 0.08 * vz

        self.last = time.time()

    def age(self):
        return time.time() - self.last
