#!/usr/bin/env python3
# ===============================================================
# TVF-ORCA Hybrid Swarm Control - Main Entry Point
# ===============================================================
# PX4 + MAVSDK + TVF-ORCA Collision Avoidance + Telemetry Noise
#
# Launches a swarm of drones with simulated telemetry impairments:
#   1. Packet drops
#   2. Burst outages
#   3. Gaussian position noise
#   4. Occasional delay jitter
#
# Good for robustness testing in SITL with multiple vehicles.
# ===============================================================

import asyncio
from src.drone import Drone


async def main():
    """
    Launch three drones in formation to their respective goals.
    
    Drone configuration:
    - Drone 1: Port 14540, Goal [12, 2, -2]
    - Drone 2: Port 14541, Goal [10, 4, -2]
    - Drone 3: Port 14542, Goal [10, 0, -2]
    """
    drones = [
        Drone(14540, [12, 2, -2]),
        Drone(14541, [10, 4, -2]),
        Drone(14542, [10, 0, -2]),
    ]

    await asyncio.gather(*(d.run() for d in drones))


if __name__ == "__main__":
    asyncio.run(main())
