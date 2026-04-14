# src/__init__.py

# This allows you to do: from src import Drone
from .DroneClass import Drone
from .utils import calculate_swarm_velocity
from .config import MAVSDK_SERVER_BIN, shared_swarm_telemetry, TAKEOFF_ALTITUDE, UPDATE_RATE_HZ

# If you have other classes later, add them here too:
# from .ControllerClass import Controller