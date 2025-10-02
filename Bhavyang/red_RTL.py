#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""

d = 2R * arcsin(sqrt(sin²(Δlat/2) + cos(lat1) * cos(lat2) * sin²(Δlon/2)))

Here, R = 6371 km = 6371000 m

"""

# d = 2*R * math.arcsin(math.sqrt((math.sin(del_lat/2))**2 + math.cos(lat1) * math.cos(lat2) * (math.sin(del_lon/2))**2))
from __future__ import print_function
import cv2
import numpy as np
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
import math

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.", default = '127.0.0.1:14550')
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
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
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    
    
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)




R = 6371000
lat_self = vehicle.location.global_relative_frame.lat
lon_self = vehicle.location.global_relative_frame.lon
lat_home = vehicle.location.global_relative_frame.lat
lon_home = vehicle.location.global_relative_frame.lon
def distance(p1_lat, p1_lon, p2_lat, p2_lon):
    p1_lat = math.radians(p1_lat)
    p2_lat = math.radians(p2_lat)
    p1_lon = math.radians(p1_lon)
    p2_lon = math.radians(p2_lon)
    del_lat = p1_lat - p2_lat
    del_lon = p1_lon - p2_lon
    dist = 2*R * math.asin(math.sqrt((math.sin(del_lat/2))**2 + math.cos(p2_lat) * math.cos(p1_lat) * (math.sin(del_lon/2))**2))
    return dist


arm_and_takeoff(5)
print("Set default/target airspeed to 30")
vehicle.airspeed = 30

video = cv2.VideoCapture(0)
print("Going towards first point for 30 seconds ...")
point1 = LocationGlobalRelative(-35.36191839, 149.16499445, 5)
# point2 = LocationGlobalRelative(-35.36249310, 149.16548709, 5)
# point3 = LocationGlobalRelative(-35.36281189, 149.16553535, 5)
print(lat_self, lon_self, distance(lat_self, lon_self, point1.lat, point1.lon))
def just_go(point, point_lat, point_lon):
    lat_self = vehicle.location.global_relative_frame.lat
    lon_self = vehicle.location.global_relative_frame.lon
    vehicle.simple_goto(point)
    while distance(lat_self, lon_self, point_lat, point_lon) >= 1:
        lat_self = vehicle.location.global_relative_frame.lat
        lon_self = vehicle.location.global_relative_frame.lon
        print(distance(lat_self, lon_self, point_lat, point_lon))
        ret, frame = video.read()
        if not ret:
            break
        h, w, channel = frame.shape
        new_frame = frame[int(h*0.5): h, 0: int(w*0.5)] # new_frame
        hsv = cv2.cvtColor(new_frame, cv2.COLOR_BGR2HSV)                                      
                                                                                            
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])
                                                                                       
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 + mask2
                                                                                           
        mask = cv2.erode(mask, None, iterations=3)
        mask = cv2.dilate(mask, None, iterations=3)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            c = max(contours, key = cv2.contourArea)                                           
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            cv2.circle(new_frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)
            print("object detected")
            vehicle.mode = VehicleMode("RTL")
        else:
            print("object not detected")
        cv2.line(frame, (0, int(h*0.5)), (int(w*0.5), int(h*0.5)), (0, 255, 0), 2) #L1
        cv2.line(frame, (int(w*0.5), w), (int(w*0.5), int(h*0.5)), (0, 255, 0), 2) #L2
        cv2.imshow("mask", mask)
        cv2.imshow("frame", frame)
        # cv2.imshow("ROD", new_frame) #Show feed
        if cv2.waitKey(1) & 0xFF == ord('q'):                                            
            break
        time.sleep(1)
just_go(point1, point1.lat, point1.lon)
# just_go(point2, point2.lat, point2.lon)
# just_go(point3, point3.lat, point3.lon)
print("Returning to Launch")
vehicle.mode = VehicleMode("RTL")

print("Close vehicle object")
vehicle.close()

if sitl:
    sitl.stop()
