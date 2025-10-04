#!/usr/bin/env python3

import time
import argparse
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal

import cv2
import numpy as np

FACTOR = 1.113195e5
EARTH_RADIUS = 6371
counter = 0


parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.",default='127.0.0.1:14550')
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


def arm_and_takeoff(target_altitude):
    """Arm the vehicle and fly to target_altitude (m)."""

    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    
    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {alt} m")
        if alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
        

arm_and_takeoff(10)

point1 = LocationGlobalRelative(-35.361354,149.165218,10)

#open camera and detect red
cap = cv2.VideoCapture(0)
print("Going to point1...")
vehicle.simple_goto(point1, groundspeed=20)

if not cap.isOpened():
    print("Could not detect camera.")
    exit()

while True:
    ret, frame = cap.read()
    
    if not ret:
        print("Could not capture frame.")
        exit()
    
    x_start, y_end = frame.shape[0]//2, frame.shape[1]//2
    detect = frame[x_start:frame.shape[0], 0:y_end]
    hsv = cv2.cvtColor(detect, cv2.COLOR_BGR2HSV)
    
    lower_red1 = np.array([0,100,100])
    upper_red1 = np.array([10,255,255])
    
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    
    lower_red2 = np.array([160,100,100])
    upper_red2 = np.array([180,255,255])
    
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    
    mask = cv2.bitwise_or(mask1, mask2)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        current_location = vehicle.location.global_relative_frame
        print(f"Red detected at lat: {current_location.lat}, lon: {current_location.lon}")
        vehicle.mode = VehicleMode("RTL")
        time.sleep(4)
        break
    
    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    if counter % 10 == 0:  # Check distance every 10 iterations to reduce frequency
        current_location = vehicle.location.global_relative_frame
        hav_dist = 2 * EARTH_RADIUS * math.asin(math.sqrt(math.sin((current_location.lat-point1.lat)/2)**2 + math.cos(current_location.lat)*math.cos(point1.lat)*math.sin((current_location.lon-point1.lon)/2)**2))
            
        dist = FACTOR*((current_location.lat-point1.lat)**2+(current_location.lon-point1.lon)**2)**0.5
            
        print(f"distance to point 1: {dist}")
        if hav_dist <= 0.1:
            print("reached point 1.")
            break
    
    counter += 1
    
print("Returning to Launch...")
vehicle.mode = VehicleMode("RTL")

print("Closing Vehicle object")

vehicle.close()
cap.release()
cv2.destroyAllWindows()
