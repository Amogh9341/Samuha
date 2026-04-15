#!/usr/bin/env python3
"""
hitl_telemetry_test.py
HITL Telemetry Test Script using HITLDrone Class

This script tests drone telemetry reception via serial/HITL connection.
It instantiates a HITLDrone and uses its test_telemetry() method to stream
and display telemetry data from the connected autopilot.

Usage:
  python hitl_telemetry_test.py --connection COM3:115200 --stream position
  python hitl_telemetry_test.py --connection COM3:115200 --stream all --duration 30
  python hitl_telemetry_test.py --connection /dev/ttyUSB0:115200 --stream attitude

Supported Streams:
  - position:  GPS position and altitude
  - attitude:  Roll, pitch, yaw angles
  - battery:   Battery voltage and remaining percentage
  - all:       All three streams simultaneously
"""

import asyncio
import argparse
import os
import sys
import logging

# Ensure local package imports work when running this script directly
sys.path.insert(0, os.path.dirname(__file__))
from src import HITLDrone


async def main(args):
    """
    Main function: instantiate HITLDrone and run telemetry test.
    
    Args:
        args: Parsed command-line arguments
    """
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    try:
        # Instantiate HITLDrone with connection details
        print(f"Initializing HITL Drone with connection: {args.connection}")
        hitl_drone = HITLDrone(
            connection_string=args.connection,
            drone_id=args.drone_id,
            target_ned=args.target  # Dummy target, not used in test_telemetry()
        )
        
        # Display drone status
        status = hitl_drone.get_status()
        print(f"\nDrone Status:")
        print(f"  ID: {status['drone_id']}")
        print(f"  Port: {status['port_path']}")
        print(f"  Baud Rate: {status['baud_rate']}")
        print(f"  HITL Mode: {status['is_hitl']}")
        print(f"\nStarting telemetry stream ({args.stream})...\n")
        
        # Run telemetry test using HITLDrone's method
        await hitl_drone.test_telemetry(
            stream_type=args.stream,
            duration=args.duration
        )
    
    except KeyboardInterrupt:
        print("\n\nTelemetry test interrupted by user.")
    except Exception as e:
        print(f"\nError during telemetry test: {e}")
        raise


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="HITL Telemetry Test using HITLDrone Class",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Test position telemetry on Windows
  python hitl_telemetry_test.py --connection COM3:115200 --stream position
  
  # Test attitude for 30 seconds on Linux
  python hitl_telemetry_test.py --connection /dev/ttyUSB0:115200 --stream attitude --duration 30
  
  # Stream all telemetry indefinitely
  python hitl_telemetry_test.py --connection COM3:115200 --stream all
        """
    )
    
    parser.add_argument(
        "--connection",
        default="COM3:115200",
        help="HITL serial connection string (default: COM3:115200)\n"
             "Format: COMx:baud (Windows) or /dev/ttyUSBx:baud (Linux)\n"
             "Baud rate defaults to 115200 if omitted"
    )
    
    parser.add_argument(
        "--drone-id",
        type=int,
        default=1,
        help="Drone identifier for logging (default: 1)"
    )
    
    parser.add_argument(
        "--stream",
        choices=["position", "attitude", "battery", "all"],
        default="position",
        help="Telemetry stream type to display (default: position)"
    )
    
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Test duration in seconds (default: indefinite until Ctrl+C)"
    )
    
    parser.add_argument(
        "--target",
        type=float,
        nargs=3,
        default=[0, 0, 0],
        metavar=("N", "E", "D"),
        help="Dummy target position [N E D] in meters (unused in test, default: 0 0 0)"
    )
    
    args = parser.parse_args()
    
    try:
        asyncio.run(main(args))
    except KeyboardInterrupt:
        print("\nExiting.")
    except Exception as e:
        print(f"Fatal error: {e}")
        sys.exit(1)
