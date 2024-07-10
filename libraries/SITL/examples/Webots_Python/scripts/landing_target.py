#!/usr/bin/env python3

#
# An example script that receives images from a WebotsArduVehicle on port 5599 
# and displays them overlayed with any ArUco markers using OpenCV.
# Requires opencv-python (`pip3 install opencv-python`)
#
import cv2
import socket
import struct
import math
import os
import numpy as np
import time
from pymavlink import mavutil

from apscheduler.schedulers.background import BackgroundScheduler

master = mavutil.mavlink_connection(device='udp:127.0.0.1:14550', source_system=1, source_component=1)

master.wait_heartbeat()
# print("Received heartbeat from the vehicle")

# Connect to WebotsArduVehicle
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("127.0.0.1", 5599))

s2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s2.connect(("127.0.0.1", 5598))


# ArUco setup
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters_create()

target_node = 0

horizontal_res = 640
vertical_res = 480
horizontal_fov = 1.085
vertical_fov = 2* math.atan(math.tan(horizontal_fov / 2) / (640/480))

header_size = struct.calcsize("=HH")
marker_size = 50

intrinsic_camera = np.array(((523.56, 0.0, 320.0), (0.0, 392.67, 240.0), (0, 0, 1.0)))
# distCoeff = np.array([-0.32,0.14,0.001,-0.0005,0.0])
distCoeffs = np.array([0.0,0.0,0.0,0.0,0.0])
time_ms = int(time.time() * 1000)

while True:
    # Receive header
    header = s.recv(header_size)
    if len(header) != header_size:
        print("Header size mismatch")
        break

    # Parse header
    width, height = struct.unpack("=HH", header)

    # Receive image
    bytes_to_read = width * height
    img = bytes()
    while len(img) < bytes_to_read:
        img += s.recv(min(bytes_to_read - len(img), 4096))

    # Convert to numpy array
    img = np.frombuffer(img, np.uint8).reshape((height, width))
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    # Detect ArUco markers
    (corners, ids, rejected) = cv2.aruco.detectMarkers(img, aruco_dict, parameters=aruco_params)
    if ids is not None:
        rvecs,tvecs,_ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, intrinsic_camera, distCoeffs)
        #print(tvecs)
        
        for i in range(len(ids)):
            if ids[i] == target_node:
                # current_time_ms = int(time.time() * 1000)-time_ms

                # Extract the corner coordinates and compute the center
                corner = corners[i][0]
                x_avg = (corner[0][0] + corner[2][0] + corner[1][0] + corner[3][0]) / 4.0
                y_avg = (corner[0][1] + corner[2][1] + corner[1][1] + corner[3][1]) / 4.0

                # Calculate angles and distance
                x_ang = (x_avg-horizontal_res*0.5)*horizontal_fov/horizontal_res
                y_ang = (y_avg-vertical_res*0.5)*vertical_fov/vertical_res

                target_number = ids[i][0]

                # Send landing target message
                master.mav.landing_target_send(
                    time_usec=int(time_ms),
                    target_num=target_number,
                    frame=mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                    angle_x=x_ang,
                    angle_y=y_ang,
                    distance = tvecs[0][0][2],
                    size_x = 0,
                    size_y=0
                )
                print([x_ang,y_ang])
        if rvecs is not None:
            for i in range(len(ids)):
                # Draw marker and axes
                cv2.aruco.drawAxis(img, intrinsic_camera, distCoeffs, rvecs[i], tvecs[i], 0.1)
                cv2.aruco.drawDetectedMarkers(img, corners)
    # Display image
    cv2.imshow("ArUco Marker Detection", img)
    if cv2.waitKey(1) == ord("q"):
        break
s.close()
cv2.destroyAllWindows()

