#!/usr/bin/env python3
"""
hitl_telemetry_test.py
Updated for NEW HITLDrone constructor.

New constructor:
HITLDrone(
    connection_string,
    drone_id,
    target_lat,
    target_lon,
    target_alt_m
)

Usage:
python hitl_telemetry_test.py --connection COM15:57600 --stream position
python hitl_telemetry_test.py --connection COM15:57600 --stream all --duration 30
"""

import asyncio
import argparse
import os
import sys
import logging

# Local import path
sys.path.insert(0, os.path.dirname(__file__))

from src import HITLDrone


async def main(args):
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    )

    try:
        print(f"Initializing HITL Drone with connection: {args.connection}")

        # Updated constructor
        drone = HITLDrone(
            connection_string=args.connection,
            drone_id=args.drone_id,
            target_lat=args.target_lat,
            target_lon=args.target_lon,
            target_alt_m=args.target_alt
        )

        # Status (if get_status exists)
        try:
            status = drone.get_status()

            print("\nDrone Status:")
            print(f"  ID        : {status['drone_id']}")
            print(f"  Port      : {status['port_path']}")
            print(f"  Baud Rate : {status['baud_rate']}")
            print(f"  HITL Mode : {status['is_hitl']}")

        except:
            print("\nDrone initialized successfully.")

        print(f"\nTarget GPS:")
        print(f"  Latitude  : {args.target_lat}")
        print(f"  Longitude : {args.target_lon}")
        print(f"  Altitude  : {args.target_alt} m")

        print(f"\nStarting telemetry stream: {args.stream}\n")

        await drone.test_telemetry(
            stream_type=args.stream,
            duration=args.duration
        )

        print("\nTelemetry test finished.")

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    except Exception as e:
        print(f"\nTelemetry test failed: {e}")
        raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="HITL Telemetry Test Script",
        formatter_class=argparse.RawTextHelpFormatter
    )

    parser.add_argument(
        "--connection",
        default="COM15:57600",
        help="Serial connection string\n"
             "Examples:\n"
             "COM15:57600\n"
             "/dev/ttyUSB0:57600"
    )

    parser.add_argument(
        "--drone-id",
        type=int,
        default=1,
        help="Drone ID"
    )

    parser.add_argument(
        "--stream",
        choices=["position", "attitude", "battery", "all"],
        default="position",
        help="Telemetry stream type"
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Duration in seconds (default infinite)"
    )

    # REQUIRED by new constructor
    parser.add_argument(
        "--target-lat",
        type=float,
        default=26.1445,
        help="Target latitude"
    )

    parser.add_argument(
        "--target-lon",
        type=float,
        default=91.7362,
        help="Target longitude"
    )

    parser.add_argument(
        "--target-alt",
        type=float,
        default=50.0,
        help="Target altitude in meters"
    )

    args = parser.parse_args()

    try:
        asyncio.run(main(args))

    except KeyboardInterrupt:
        print("\nExited.")

    except Exception as e:
        print(f"Fatal error: {e}")
        sys.exit(1)