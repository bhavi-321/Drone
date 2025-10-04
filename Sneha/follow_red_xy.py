#!/usr/bin/env python3

import time
import argparse
import math
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil

import cv2
import numpy as np


parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.",
                    default='127.0.0.1:14550')
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

def condition_yaw(heading, is_relative,direction):
    
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
    heading: yaw in degrees, 0-360
    is_relative: 1 for relative yaw, 0 for absolute angle
    direction: -1 for counter-clockwise, 1 for clockwise, 0 for shortest path
    """

    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,                                  # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0,                                     #confirmation
        heading,                               # param 1, yaw in degrees
        0,                                     # param 2, yaw speed deg/s
        direction,                             # param 3, direction -1 ccw, 1 cw
        is_relative,                           # param 4, relative offset 1, absolute angle 0
        0, 0, 0)                               # param 5 ~ 7 not used

    vehicle.send_mavlink(msg)                  #send command to vehicle

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.
    velocity_x: velocity in m/s along North direction
    velocity_y: velocity in m/s along East direction
    velocity_z: velocity in m/s along Down direction (positive downward)
    """

    # create the SET_POSITION_TARGET_LOCAL_NED command
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

# Initialize OpenCV video capture
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Could not detect camera.")
    vehicle.mode = VehicleMode("RTL")
    vehicle.close()
    exit()

print("Starting red object tracking...")

while True:
    ret, frame = cap.read()
    
    if not ret:
        print("Could not capture frame.")
        exit()

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # create masks for red color
    lower_red1 = np.array([0,100,100])
    upper_red1 = np.array([10,255,255])
    
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    
    lower_red2 = np.array([160,100,100])
    upper_red2 = np.array([180,255,255])
    
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    
    mask = cv2.bitwise_or(mask1, mask2)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)
    
    # find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if contours:
        c = max(contours, key=cv2.contourArea)           # find the largest contour
        (x, y), radius = cv2.minEnclosingCircle(c)       # get the minimum enclosing circle around the largest contour
        cx , cy = frame.shape[1]//2, frame.shape[0]//2
        error_x = x - cx                                 # calculate error in x direction
        error_y = y - cy                                 # calculate error in y direction

        if abs(error_x) > radius:
            # if the error is significant, adjust yaw
            if error_x < 0:
                print("Turn left")
                condition_yaw(4, is_relative=1, direction=-1)
            else:
                print("Turn right")
                condition_yaw(4, is_relative=1, direction=1)
        
        if abs(error_y) > radius:
            # if the error is significant, adjust altitude
            if error_y < 0:
                print("Move up")
                send_ned_velocity(0, 0, -0.2, 1)
                
            else:
                print("Move down")
                send_ned_velocity(0, 0, 0.2, 1)
        
        cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)  # draw the circle around the detected red object
    
    # Display the resulting frame and mask
    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break 
    
# clean up the code after exiting the loop    
print("Returning to Launch...")
vehicle.mode = VehicleMode("RTL")

vehicle.close()
cap.release()
cv2.destroyAllWindows() 