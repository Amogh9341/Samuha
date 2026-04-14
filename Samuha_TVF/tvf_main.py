import asyncio
from src import Drone

async def main():
    """
    TVF (Tangential Vector Field) Main Entry Point
    
    Demonstrates swarm flight with:
    - Repulsive forces for collision avoidance
    - Attractive forces toward goal
    - Tangential vector field for swarm organization
    - Telemetry noise and packet loss simulation
    """

    drones = [
        Drone(14540, [12, 2, -2]),
        Drone(14541, [10, 4, -2]),
        Drone(14542, [10, 0, -2]),
    ]

    await asyncio.gather(*(d.run() for d in drones))

if __name__ == "__main__":
    asyncio.run(main())
