#!/usr/bin/env python3
# ===============================================================
# TVF-ORCA Hybrid Swarm Control - Main Entry Point
# ROS2 Compatible Version
# ===============================================================

import asyncio
import rclpy

from src.drone import Drone


async def main():
    """
    Launch three drones in formation to goals.
    """

    # -----------------------------------------------------------
    # REQUIRED BEFORE CREATING Drone(Node) OBJECTS
    # -----------------------------------------------------------
    rclpy.init()

    drones = []

    try:
        drones = [
            Drone(14540, [10, 2, -2]),
            Drone(14541, [10, 8, -2]),
            Drone(14542, [10, 5, -2]),
        ]

        await asyncio.gather(
            *(d.run() for d in drones)
        )

    finally:
        # -------------------------------------------------------
        # CLEANUP NODES
        # -------------------------------------------------------
        for d in drones:
            try:
                d.destroy_node()
            except:
                pass

        rclpy.shutdown()


if __name__ == "__main__":
    asyncio.run(main())