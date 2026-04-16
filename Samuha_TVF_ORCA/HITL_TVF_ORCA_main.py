#!/usr/bin/env python3
# ===============================================================
# TVF-ORCA Hardware-In-The-Loop (HITL) - Main Entry Point
# ===============================================================
# PX4 + MAVSDK + TVF-ORCA Collision Avoidance with Real Drone Hardware
# 
# Launches a swarm of drones with ACTUAL HARDWARE via USB connection
# GPS coordinates and drone configurations are hardcoded below.
# ===============================================================

import asyncio
import logging
import sys
import platform
from src import HITLDrone


# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - [%(name)s] - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


async def main():
    """
    Launch drone swarm in Hardware-In-The-Loop mode.
    
    Drone configuration with GPS targets:
    - Drone 1: COM15:57600, Target GPS (10.124675, 10.2407543, 0m)
    - Drone 2: COM4:115200, Target GPS (10.125000, 10.241000, 0m) (uncomment to enable)
    - Drone 3: COM5:115200, Target GPS (10.124500, 10.240500, 0m) (uncomment to enable)
    
    To add/modify drones:
    1. Add a new HITLDrone() line with:
       - connection_string: "COMx:57600" or "COMx:115200" (Windows) 
       - connection_string: "/dev/ttyUSBx:115200" (Linux)
       - drone_id: Unique identifier (1, 2, 3, ...)
       - target_lat: Target latitude in degrees
       - target_lon: Target longitude in degrees
       - target_alt_m: Target altitude in meters (MSL)
    
    Note: GPS coordinates are automatically converted to NED relative to the
    drone's home position (set by PX4 on first GPS lock).
    """
    drones = [
        HITLDrone("COM15:57600", drone_id=1, target_lat=10.124675, target_lon=10.2407543, target_alt_m=0),
        HITLDrone("COM4:115200", drone_id=2, target_lat=10.125000, target_lon=10.241000, target_alt_m=0),
        HITLDrone("COM5:115200", drone_id=3, target_lat=10.124500, target_lon=10.240500, target_alt_m=0),
    ]

    logger.info(f"Starting HITL swarm with {len(drones)} drone(s)")
    
    try:
        # Run all drones concurrently
        await asyncio.gather(*(d.run() for d in drones))
        logger.info("HITL swarm completed successfully")
    except Exception as e:
        logger.error(f"HITL swarm error: {e}", exc_info=True)
        raise


if __name__ == "__main__":
    # Windows asyncio event loop policy
    if platform.system() == "Windows":
        asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())
        logger.info("Using Windows selector event loop policy")
    
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("HITL swarm interrupted by user")
        sys.exit(0)
    except Exception as e:
        logger.critical(f"Fatal error: {e}", exc_info=True)
        sys.exit(1)
