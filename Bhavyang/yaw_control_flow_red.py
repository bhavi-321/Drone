#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2015-2016, 3D Robotics.
guided_set_speed_yaw.py: (Copter Only)

This example shows how to move/direct Copter and send commands in GUIDED mode using DroneKit Python.

Example documentation: http://python.dronekit.io/examples/guided-set-speed-yaw-demo.html
"""
from __future__ import print_function
import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
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
arm_and_takeoff(1.5)
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

while True:
    ret, frame = video.read()
    if not ret:
        break
    h, w, channel = frame.shape
    # print(frame.shape)
    new_frame = frame[int(h*0.5): h, 0: int(w*0.5)] # new_frame
    hsv = cv2.cvtColor(new_frame, cv2.COLOR_BGR2HSV)                                      
                                                                                            
    lower_red1 = np.array([180, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
                                                                                       
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 + mask2
                                                                                           
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        c = max(contours, key = cv2.contourArea)                                           
        ((x, y), radius) = cv2.minEnclosingCircle(c)
        cv2.circle(new_frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
        # print(x, y)
        print(dist_2_angle(x, y, center_25, w, h), vehicle.heading)
        controll_yaw(dist_2_angle(x, y, center_25, w, h))
        # print("object detected")
    center_25 = (int(w*0.25), int(h*0.75))
    cv2.line(frame, (0, int(h*0.5)), (int(w*0.5), int(h*0.5)), (0, 255, 0), 2) #L1
    cv2.line(frame, (int(w*0.5), w), (int(w*0.5), int(h*0.5)), (0, 255, 0), 2) #L2
    cv2.circle(frame, center_25, int(2), (0, 0, 255), 2)
    
    cv2.imshow("mask", mask)
    cv2.imshow("frame", frame)
    # cv2.imshow("ROD", new_frame) #Show feed
    if cv2.waitKey(1) & 0xFF == ord('q'):                                            
        break

video.release()
cv2.destroyAllWindows()
print("Close vehicle object")
vehicle.close()

if sitl:
    sitl.stop()
