# ===============================================================
# CONFIG (BALANCED FAST + SMOOTH VERSION)
# ===============================================================

MAVSDK_SERVER_BIN = "/home/doffy_ubjammy/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server"

UPDATE_RATE_HZ = 18
DT = 1.0 / UPDATE_RATE_HZ

TAKEOFF_ALTITUDE = 2.0
MAX_HEIGHT_M = 4.0

# -------------------------------------------------
# Faster motion
# -------------------------------------------------
MAX_SPEED_XY = 2.75
MAX_SPEED_Z  = 0.42

# -------------------------------------------------
# Stronger goal seeking
# -------------------------------------------------
GOAL_GAIN = 1.25

# -------------------------------------------------
# Better avoidance while fast
# -------------------------------------------------
REPULSION_GAIN  = 2.5
TANGENTIAL_GAIN = 1.18

SAFE_RADIUS = 1.2
INFLUENCE_RADIUS = 2.2

GOAL_TOLERANCE = 0.75
TELEMETRY_TIMEOUT = 3

# -------------------------------------------------
# Smooth but more responsive
# -------------------------------------------------
VEL_ALPHA = 0.32

MAX_ACCEL_XY = 1.5
MAX_ACCEL_Z  = 0.45

CMD_DEADBAND = 0.015

# ===============================================================
# TELEMETRY IMPAIRMENT MODEL
# Moderate stress, still flyable
# ===============================================================

DROP_RATE = 0.5

BURST_START_PROB = 0.05
BURST_MIN = 5
BURST_MAX = 16

NOISE_STD_XY = 0.22
NOISE_STD_Z  = 0.10

DELAY_PROB = 0.5
DELAY_MIN = 0.06
DELAY_MAX = 0.335

shared_swarm_telemetry = {}