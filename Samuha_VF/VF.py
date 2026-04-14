# main.py
import asyncio
import argparse
from src.DroneClass import Drone

async def main():
    parser = argparse.ArgumentParser(description="Samuha Drone Swarm Controller")
    parser.add_argument('--ports', nargs='+', type=int, default=[14540, 14541, 14542], help='UDP ports for drones')
    args = parser.parse_args()

    # Define targets dynamically based on number of ports
    # This creates a simple line of targets 20m out
    targets = [[5, i * 0.3, -10] for i in range(len(args.ports))]
    
    swarm = [Drone(port, targets[i]) for i, port in enumerate(args.ports)]
    
    print(f"Starting swarm with {len(swarm)} drones...")
    await asyncio.gather(*(d.run() for d in swarm))

if __name__ == "__main__":
    asyncio.run(main())