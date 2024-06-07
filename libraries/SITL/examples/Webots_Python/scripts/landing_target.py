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
os.environ['MAVLINK20'] = '1'
master = mavutil.mavlink_connection(device='udpout:127.0.0.1:14550',source_system=1,source_component=1)

# connect to WebotsArduVehicle
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect(("127.0.0.1", 5599))

# ArUco setup
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
aruco_params = cv2.aruco.DetectorParameters_create()

header_size = struct.calcsize("=HH")

while True:
    # receive header
    header = s.recv(header_size)
    if len(header) != header_size:
        print("Header size mismatch")
        break

    # parse header
    width, height = struct.unpack("=HH", header)

    # for CV applications we may want camera intrinsics such as focal length: 
    # https://stackoverflow.com/questions/61555182/webot-camera-default-parameters-like-pixel-size-and-focus
    # cam_focal_length = 2 * np.arctan(np.tan(cam_fov * 0.5) / (cam_width / cam_height))

    # receive image
    bytes_to_read = width * height
    img = bytes()
    while len(img) < bytes_to_read:
        img += s.recv(min(bytes_to_read - len(img), 4096))

    # convert to numpy array
    img = np.frombuffer(img, np.uint8).reshape((height, width))

    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    cx = width/2
    cy = height/2
    shape= [width,height]
    w=width
    h=height
    target_node = 0
    
    intrinsic_camera = np.array(((394.08232205, 0, 319.53102932), (0, 394.27775824, 217.45178432), (0, 0, 1)))
    distCoeffs = np.array([-0.01271642,  0.03904447, -0.00020002,  0.00012752, -0.04279091])

    # detect ArUco markers
    (corners, ids, rejected) = cv2.aruco.detectMarkers(img, aruco_dict, parameters=aruco_params)
    if ids is not None:
        rvecs,tvecs,_ = cv2.aruco.estimatePoseSingleMarkers(corners, 0.5, intrinsic_camera, distCoeffs)
        tvecs = tvecs[0]
        for i in range(len(ids)):
            if ids[i] == target_node:
                current_time_ms = int(time.time() * 1000)

                # Extract the corner coordinates and compute the center
                corner = corners[i][0]
                center_x = (corner[0][0] + corner[2][0]) / 2
                center_y = (corner[0][1] + corner[2][1]) / 2

                # Assuming some camera calibration parameters and distance measurement (dummy values here)
                angx = np.arctan2(tvecs[i][0], tvecs[i][2])
                angy = np.arctan2(tvecs[i][1], tvecs[i][2])
                dist = np.linalg.norm([tvecs[i][0],tvecs[i][1]])

                sizex = 0.2/dist
                sizey = 0.2/dist
                target_number = ids[i][0]
                master.mav.landing_target_send(
                    time_usec=current_time_ms,
                    target_num=target_number,
                    frame=mavutil.mavlink.MAV_FRAME_BODY_FRD,
                    angle_x=angx,
                    angle_y=angy,
                    distance=dist,
                    size_x=sizex,
                    size_y=sizey
                )
                print(tvecs[i][0], tvecs[i][1], tvecs[i][2])

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
