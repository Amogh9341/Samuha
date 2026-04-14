# ===============================================================
# TVF-ORCA Hybrid Flight Configuration
# ===============================================================

# =================================================================
# SYSTEM & INFRASTRUCTURE
# =================================================================

MAVSDK_SERVER_BIN = "/home/doffy_ubjammy/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server"
SDK_PORT_OFFSET = 40000

# =================================================================
# CONTROL LOOP & TIMING
# =================================================================

UPDATE_RATE_HZ = 18
DT = 1.0 / UPDATE_RATE_HZ

# Async operation sleep intervals
HEALTH_CHECK_SLEEP = 1.0
TAKEOFF_WAIT_TIME = 8.0
PREDICT_SLEEP = 0.1
TELEMETRY_STOP_SLEEP = 0.2

# Filter timing constraints
MINIMUM_DT = 0.01

# =================================================================
# FLIGHT PHYSICS & CONSTRAINTS
# =================================================================

# Altitude management (meters)
TAKEOFF_ALTITUDE = 2.0
MAX_HEIGHT_M = 4.0

# Speed limits (m/s)
MAX_SPEED_XY = 2.75
MAX_SPEED_Z = 0.42

# Acceleration limits (m/s²)
MAX_ACCEL_XY = 2.2
MAX_ACCEL_Z = 0.45

# =================================================================
# TELEMETRY & STATE ESTIMATION
# =================================================================

# Kalman filter confidence
TELEMETRY_TIMEOUT = 2.8

# Velocity estimate smoothing (low-pass filter coefficients)
# North and East axes (moderately responsive)
KF_VEL_NE_DECAY = 0.88
KF_VEL_NE_UPDATE = 0.12

# Vertical axis (more conservative, smoother)
KF_VEL_D_DECAY = 0.92
KF_VEL_D_UPDATE = 0.08

# =================================================================
# TELEMETRY IMPAIRMENT SIMULATION
# =================================================================

# Packet loss model
DROP_RATE = 0.10

# Burst outages (simulating radio interference)
BURST_START_PROB = 0.035
BURST_MIN = 5
BURST_MAX = 16

# Measurement noise (Gaussian, meters)
NOISE_STD_XY = 0.22
NOISE_STD_Z = 0.10

# Transmission delay jitter
DELAY_PROB = 0.10
DELAY_MIN = 0.04
DELAY_MAX = 0.28

# =================================================================
# COLLISION AVOIDANCE - TVF CORE
# =================================================================

# Attraction to goal
GOAL_GAIN = 1.25
GOAL_TOLERANCE = 0.75

# Repulsion from obstacles
REPULSION_GAIN = 2.1
TANGENTIAL_GAIN = 1.18
SAFE_RADIUS = 1.5
INFLUENCE_RADIUS = 2.5

# =================================================================
# COLLISION AVOIDANCE - FINE TUNING PARAMETERS
# =================================================================

# Numerical stability
DIST_EPSILON = 1e-6
MIN_DIST_CLAMP = 0.25

# Vertical velocity handling
VERT_VEL_SCALE = 0.45
VERT_REPULSION_GAIN = 0.12

# Field influence dynamics
INFLUENCE_POWER = 3
SAFE_RADIUS_SCALE = 1.4
TANGENT_SCALE_DIV = 2.0

# Emergency reciprocal avoidance (ORCA-like layer)
EMERGENCY_PUSH_MULT = 2.8
TANGENTIAL_PASS_MULT = 1.1
URGENCY_AGGRESSION_REDUCE = 0.35

# =================================================================
# NAVIGATION & CONTROL FILTERING
# =================================================================

# Velocity command filtering (low-pass smoothness)
VEL_ALPHA = 0.32

# Command deadband (m/s - prevents jitter)
CMD_DEADBAND = 0.015
