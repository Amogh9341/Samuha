import socket

def check_drone_ports(start_port=14540, end_port=14560):
    print(f"Scanning UDP ports {start_port} to {end_port} for heartbeats...")
    active_ports = []

    for port in range(start_port, end_port + 1):
        # Create a UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.settimeout(1.0) # Wait 1 second for a heartbeat
        
        try:
            # Bind to all interfaces on this port
            sock.bind(('0.0.0.0', port))
            
            print(f"Listening on port {port}...", end="\r")
            data, addr = sock.recvfrom(1024)
            
            # MAVLink 2.0 packets start with 0xFD
            # MAVLink 1.0 packets start with 0xFE
            if data[0] in [0xFD, 0xFE]:
                print(f"[FOUND] Drone detected on Port: {port} from {addr[0]}")
                active_ports.append(port)
            
        except socket.timeout:
            pass # No data received
        except Exception as e:
            # Usually means the port is already in use by another script/QGC
            if "[Errno 98]" in str(e) or "[Errno 10048]" in str(e):
                print(f"[BUSY]  Port {port} is already being used (Active Drone)")
                active_ports.append(port)
        finally:
            sock.close()

    print("\n--- Scan Complete ---")
    print(f"Active MAVLink Ports: {active_ports}")
    return active_ports

if __name__ == "__main__":
    check_drone_ports()