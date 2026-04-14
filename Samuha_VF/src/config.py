# ============================
# config.py (EDITS)
# ============================

MAVSDK_SERVER_BIN = "/home/doffy_ubjammy/.local/lib/python3.10/site-packages/mavsdk/bin/mavsdk_server"

# Physics & Swarm Parameters
BUFFER_RADIUS = 0.5

# --- NEW PID gains for repulsion ---
REPULSION_KP = 1
REPULSION_KI = 0.05
REPULSION_KD = 0.5

TARGET_SPEED = 1
UPDATE_RATE_HZ = 20
TAKEOFF_ALTITUDE = 2
MAX_HEIGHT_M = 4

shared_swarm_telemetry = {}

ATTRACT_KP_XY = 1.25
ATTRACT_KI_XY = 0.04
ATTRACT_KD_XY = 0.55

ATTRACT_KP_Z  = 0.80
ATTRACT_KI_Z  = 0.02
ATTRACT_KD_Z  = 0.30