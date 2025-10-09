#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
from pyzbar.pyzbar import decode
import time
import math
import argparse  
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.",  default = '127.0.0.1:14550')
args = parser.parse_args()
connection_string = args.connect
sitl = None
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

def dist_2_angle(x, y, center_25, w, h):
    distance = (((center_25[0]-x))**2 + ((center_25[1] - y))**2)**0.5
    distance_x_axis = center_25[0]-x
    angle = (180/center_25[0])*distance_x_axis
    real_angle = angle
    return angle
def dist_2_dist_y(x, y, center_25, w, h):
    distance_y_axis = center_25[1]-y
    return distance_y_axis
def arm_and_takeoff(aTargetAltitude):

    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)        
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

#Arm and take of to altitude of 5 meters
arm_and_takeoff(3)
video = cv2.VideoCapture(0)
def controll_yaw(angle, relative=False):
    if relative:
        is_relative = 1
    else:
        is_relative = 0
    if angle >= 0:
        angle = abs(angle)
    else:
        angle = 360 + angle
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, abs(angle), 0, 1, is_relative, 0, 0, 0)
    vehicle.send_mavlink(msg)
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,                                     # time_boot_ms (not used)
        0, 0,                                  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,   # frame
        0b0000111111000111,                    # type_mask (only speeds enabled)
        0, 0, 0,                               # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,    # x, y, z velocity in m/s
        0, 0, 0,                               # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)                                  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
def check_qr(image_path):
    img = cv2.imread(image_path)
    qr_codes = decode(img)
    if qr_codes:
        for qr in qr_codes:
            check_qr_data = qr.data.decode('utf-8')
            # print(qr_data)
            
    return check_qr_data

target_qr = check_qr("//media//bhavi//2278E09278E0664F//DroneKit_mod_codes//image_qr_check.jpg")
while True:
    ret, frame = video.read()
    h, w, channel = frame.shape
    frame_center = (int(w/2), int(h/2))
    if not ret:
        break
    for qr in decode(frame):
        qr_data = qr.data.decode('utf-8')
        # print(qr_data)
        pts = qr.polygon
        pts = np.array([(pt.x, pt.y) for pt in pts])
        pts = pts.reshape((-1, 1, 2)) 
        cv2.polylines(frame, [np.array(pts)], True, (0,255,0), 2)
        M = cv2.moments(pts)
        if M['m00'] != 0:
            qr_x = int(M['m10'] / M['m00'])
            qr_y = int(M['m01'] / M['m00'])
            cv2.circle(frame, (qr_x, qr_y), int(2), (0, 0, 255), 2)

        if qr_data == target_qr:
            print("Target QR code detected!")
            controll_yaw(dist_2_angle(qr_x, qr_y, frame_center, w, h))
            if 0<= qr_y <= 240:
                # print(dist_2_dist_y(x, y, center_25, w, h), center_25, y)
                print(vehicle.heading, vehicle.location.global_relative_frame.alt)
                send_ned_velocity(0, 0, -0.2, 1)
            else:
                print(vehicle.heading, vehicle.location.global_relative_frame.alt)
                send_ned_velocity(0, 0, 0.2, 1)
            # print("object detected")
            # print(pts)
        else:
            print("Wrong QR")
    
    cv2.circle(frame, frame_center, int(2), (0, 0, 255), 2)
    cv2.imshow('QR Code Scanner', frame)
    # cv2.imshow("qr_data", qr_data)
     
    if cv2.waitKey(1) & 0xFF == ord('q'):
        # webbrowser.open(qr_data)
        break
video.release()
cv2.destroyAllWindows()

video.release()
cv2.destroyAllWindows()
print("Close vehicle object")
vehicle.close()

if sitl:
    sitl.stop()
