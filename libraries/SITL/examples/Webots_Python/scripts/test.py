import time
import os
from pymavlink import mavutil

# Set environment variables for MAVLink2
os.environ['MAVLINK_DIALECT'] = 'ardupilotmega'
os.environ['MAVLINK09'] = '1'
os.environ['MAVLINK20'] = '0'

# Connect to ArduPilot
connection = mavutil.mavlink_connection('udp:127.0.0.1:14550',source_system=255)

# Wait for the heartbeat message to find the system ID
#connection.wait_heartbeat()
#print("Heartbeat from system (system %u component %u)" % (connection.target_system, connection.target_component))

def send_system_time():
    # Get the current time
    unix_time = int(time.time())  # Current UNIX time in milliseconds
    boot_time = int(time.time())  # Time since system boot in milliseconds
    print(unix_time, boot_time)
    
    # Send SYSTEM_TIME message
    connection.mav.system_time_send(
        time_unix_usec=unix_time * 1000,  # UNIX epoch time in microseconds
        time_boot_ms=boot_time            # Time since system boot in milliseconds
    )

# Main loop
while True:
    send_system_time()
    time.sleep(1)  # Send system time every 1 second

