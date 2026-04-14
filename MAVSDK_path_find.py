import os
import sys
import site

# This automatically finds where pip installed the mavsdk_server binary
def get_mavsdk_server_path():
    # Check common locations
    user_site = site.getusersitepackages()
    system_site = site.getsitepackages()[0]
    
    # Common path suffix in the package
    suffix = "mavsdk/bin/mavsdk_server"
    
    paths_to_check = [
        os.path.join(user_site, suffix),
        os.path.join(system_site, suffix),
        "/usr/local/bin/mavsdk_server" # Just in case
    ]
    
    for path in paths_to_check:
        if os.path.exists(path):
            return path
    
    # Fallback: If not found, you might need to install it manually
    return "mavsdk_server" 

MAVSDK_SERVER_BIN = get_mavsdk_server_path()
print(f"Using MAVSDK Server at: {MAVSDK_SERVER_BIN}")