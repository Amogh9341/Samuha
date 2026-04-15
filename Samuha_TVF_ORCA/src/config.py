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
TAKEOFF_ALTITUDE = 1.0
MAX_HEIGHT_M = 3.0

# Speed limits (m/s)
MAX_SPEED_XY = 2.65
MAX_SPEED_Z = 0.42

# Acceleration limits (m/s²)
MAX_ACCEL_XY = 2.4
MAX_ACCEL_Z = 0.45

# =================================================================
# TELEMETRY & STATE ESTIMATION (STABILITY MAINTAINED)
# =================================================================

# Kalman filter confidence
TELEMETRY_TIMEOUT = 1.2

# Velocity estimate smoothing
KF_VEL_NE_DECAY = 0.94
KF_VEL_NE_UPDATE = 0.06

# Vertical axis
KF_VEL_D_DECAY = 0.95
KF_VEL_D_UPDATE = 0.05

# =================================================================
# TELEMETRY IMPAIRMENT SIMULATION (25% RSSI - POOR SIGNAL)
# =================================================================

# Random packet loss
DROP_RATE = 0.1

# Burst outages / fading
BURST_START_PROB = 0.06
BURST_MIN = 5
BURST_MAX = 18

# Position noise (meters)
NOISE_STD_XY = 0.2
NOISE_STD_Z  = 0.1

# Delay jitter
DELAY_PROB = 0.16
DELAY_MIN  = 0.03
DELAY_MAX  = 0.3

# =================================================================
# COLLISION AVOIDANCE - TVF CORE (TUNED FOR PROXIMITY)
# =================================================================

# Attraction to goal - Increased to pull drones together toward target
GOAL_GAIN = 2.25
GOAL_TOLERANCE = 0.5

# Repulsion from obstacles - Decreased to allow closer proximity
REPULSION_GAIN = 1.35
TANGENTIAL_GAIN = 0.85
SAFE_RADIUS = 0.65
INFLUENCE_RADIUS = 1.2

# =================================================================
# COLLISION AVOIDANCE - FINE TUNING PARAMETERS
# =================================================================

# Numerical stability
DIST_EPSILON = 1e-6
MIN_DIST_CLAMP = 0.20

# Vertical velocity handling
VERT_VEL_SCALE = 0.45
VERT_REPULSION_GAIN = 0.12

# Field influence dynamics
INFLUENCE_POWER = 3
SAFE_RADIUS_SCALE = 1.2
TANGENT_SCALE_DIV = 2.5

# Emergency reciprocal avoidance
EMERGENCY_PUSH_MULT = 2.1
TANGENTIAL_PASS_MULT = 1.0
URGENCY_AGGRESSION_REDUCE = 0.25

# =================================================================
# NAVIGATION & CONTROL FILTERING
# =================================================================

# Velocity command filtering
VEL_ALPHA = 0.45

# Command deadband (m/s - prevents jitter)
CMD_DEADBAND = 0.015
