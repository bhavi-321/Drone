#!/usr/bin/env python3

'''
1) drone scans for initial qr to get target location
2) drone takes off to a specified altitude
3) drone goes to the target location
4) camera feed begins 5m before reaching the coordinate
5) drone looks for a specific QR code
6) if QR code is not found, drone moves 5m ahead to search for QR code
7) if QR code is not found, drone lands at target location and RTLs to home
8) if QR code is found, drone uses region-based control to center itself above the QR code
9) once centered, drone descends to 1m above the QR code (MODIFIED)
10) after reaching 1m, drone RTLs to original home location (MODIFIED)
'''


# IMPORT REQUIRED LIBRARIES
import time
import math
import argparse
import cv2
from pyzbar.pyzbar import decode
from PIL import Image
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal
from pymavlink import mavutil
import rospy
from cv_bridge import CvBridge
import threading
from sensor_msgs.msg import Image

# DEFINE CONSTANTS
FACTOR = 1.113195e5
EARTH_RADIUS = 6371
# OG_HOME = None  # Original home location (to be set later)
DETECTED = False
TARGET_LAT = -35.36281062
TARGET_LON = 149.16515042
TARGET_ALT = 5
SPEED = 1
CONNECTION_STRING = '/dev/ttyACM0'


in_area = 10
out_area = 100
b_vel = 0.3
less_vel = 0.15


# CONNECT THE VEHICLE
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.",
                    default=CONNECTION_STRING)
args = parser.parse_args()

connection_string = args.connect

# sitl = None

# # Start SITL if no connection string specified
# if not connection_string:
#     import dronekit_sitl
#     sitl = dronekit_sitl.start_default()
#     connection_string = sitl.connection_string()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True, baud=921600)


# ARM AND TAKEOFF FUNCTION
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


# SEND BODY NED VELOCITY FUNCTION
def send_local_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
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
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,    # frame
        0b0000111111000111,                    # type_mask (only speeds enabled)
        0, 0, 0,                               # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,    # x, y, z velocity in m/s
        0, 0, 0,                               # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)                                  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    vehicle.send_mavlink(msg)
    vehicle.flush()


# YAW FUNCTION
def condition_yaw(heading, is_relative, direction, duration=2):
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

    for i in range(duration):
        vehicle.send_mavlink(msg)
        time.sleep(0.1)
    
    vehicle.flush()


# def get_frame():
#     """Get the current frame from Gazebo camera"""
#     with frame_lock:
#         if current_frame is not None:
#             return True, current_frame.copy()
#         else:
#             return False, None


def get_qr_angle(qr):
    """Calculate QR code orientation angle"""
    pts = np.array([(pt.x, pt.y) for pt in qr.polygon])
    dx, dy = pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]
    return np.degrees(np.arctan2(dy, dx))


def calculate_region_velocity(error_x, error_y):
    """
    Calculate velocity based on error regions.
    Returns (vx, vy) velocities in m/s
    
    Regions:
    - Center region (< in_area): No movement (0, 0)
    - Middle region (in_area to out_area): Fine velocity
    - Outer region (> out_area): Base velocity
    """
    
    error_distance = math.sqrt(error_x**2 + error_y**2)
    
    if error_distance < in_area:
        return 0, 0, "CENTERED"
    
    dir_x = error_x / error_distance if error_distance > 0 else 0
    dir_y = error_y / error_distance if error_distance > 0 else 0
    
    if error_distance < out_area:
        vx = dir_x * less_vel
        vy = dir_y * less_vel
        return vx, vy, "FINE_ADJUST"
    
    else:
        vx = dir_x * b_vel
        vy = dir_y * b_vel
        return vx, vy, "COARSE_ADJUST"



# ARM AND TAKEOFF TO A SPECIFIED ALTITUDE
arm_and_takeoff(TARGET_ALT)
vehicle.groundspeed = SPEED

# GO TO A SPECIFIC COORDINATE
point1 = LocationGlobalRelative(TARGET_LAT, TARGET_LON, TARGET_ALT)
vehicle.simple_goto(point1)
print("Searching for QR code...")

# WAIT FOR CAMERA FEED
cap = cv2.VideoCapture("libcamerasrc ! video/x-raw,width=1280,height=720,framerate=30/1 ! videoconvert ! appsink", cv2.CAP_GSTREAMER)

if not cap.isOpened():                                         # complete mission if camera not detected
    print("Could not detect camera.")
    print("Going to target location without QR detection...")

    while True:
        current_location = vehicle.location.global_relative_frame
        dist = FACTOR*((current_location.lat-point1.lat)**2+(current_location.lon-point1.lon)**2)**0.5
        
        print(f"distance to target location: {round(dist,2)} m")
        if dist <= 1:
            print("Reached target location.")
            break
        time.sleep(1)

    while vehicle.location.global_relative_frame.alt > 1:
        send_local_ned_velocity(0,0,1,1)
        break          
    
    print("Returning to Launch")
    vehicle.mode = VehicleMode("RTL")
    time.sleep(2)
    vehicle.close()
    
    exit()

else:
    print("Searching for QR code...")


qrs = None

while not qrs and not DETECTED:        
    ret, frame = cap.read()

    if not ret:
        print("Could not capture frame.")
        time.sleep(0.1)
        continue

    qrs = decode(frame)

    if qrs:
        print(f"Detected QR code with data: {qrs[0].data.decode('utf-8')}")
        print("QR code detected. Centering above QR code...")
        DETECTED = True
        break

    current_location = vehicle.location.global_relative_frame
    dist = FACTOR*((current_location.lat-point1.lat)**2+(current_location.lon-point1.lon)**2)**0.5
    
    print(f"distance : {round(dist,2)} m")

    if dist <= 5:
        break

    cv2.imshow("Region-Based QR Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


while not qrs and not DETECTED:        
    ret, frame = cap.read()

    if not ret:
        print("Could not capture frame.")
        time.sleep(0.1)
        continue

    qrs = decode(frame)

    if qrs:
        print(f"Detected QR code with data: {qrs[0].data.decode('utf-8')}")
        print("QR code detected. Centering above QR code...")
        DETECTED = True
        break

    send_local_ned_velocity(SPEED, 0, 0, 1)
    current_location = vehicle.location.global_relative_frame
    dist = FACTOR*((current_location.lat-point1.lat)**2+(current_location.lon-point1.lon)**2)**0.5
    print(f"distance : {round(dist,2)} m")

    if dist > 5:                         # if drone has moved more than 5m ahead of target point without finding QR
        cv2.destroyAllWindows()
        print("Could not find QR code, landing at target location and returning home...")
        vehicle.simple_goto(point1)
        time.sleep(5)
        while vehicle.location.global_relative_frame.alt >= 1:
            send_local_ned_velocity(0,0,1,1)
            print("again reached the 1m height")
        vehicle.mode = VehicleMode("RTL")
        time.sleep(2)
        vehicle.close()

        exit()

    else:
        cv2.imshow("Region-Based QR Tracking", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        continue

cv2.destroyAllWindows()

vehicle.mode = VehicleMode("GUIDED")
condition_yaw(vehicle.heading, is_relative=0, direction=0)

# REGION-BASED CENTERING LOOP
print("Starting region-based centering...")
while True:

    ret, frame = cap.read()
    if not ret:
        print("Could not capture frame.")
        time.sleep(0.1)
        continue

    qrs = decode(frame)

    if qrs:
        vertices = qrs[0].polygon

        cx, cy = frame.shape[1]//2, frame.shape[0]//2   # centre of frame
        # x = int((vertices[0][0] + vertices[1][0]) // 2) # Use mean for robustness
        # y = int((vertices[0][1] + vertices[3][1]) // 2) # Use mean for robustness
        x = int(np.mean([p.x for p in qrs[0].polygon]))
        y = int(np.mean([p.y for p in qrs[0].polygon]))
        error_x = x - cx
        error_y = y - cy

        # Calculate velocities based on regions
        vx, vy, region = calculate_region_velocity(error_x, error_y)
        
        # Check if centered
        if region == "CENTERED":
            print("QR code centered!")
            send_local_ned_velocity(0, 0, 0, 1)
            break
        
        # Invert both directions to move toward QR (negative error means move in that direction)
        # vx = -vx 
        vy = -vy
        send_local_ned_velocity(vy, vx, 0, 1)
        
        print(f"Error: {math.sqrt(error_x**2 + error_y**2):.2f} | vx: {vx:.2f}, vy: {vy:.2f}")

        # Visualize
        cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        cv2.line(frame, (cx, cy), (x, y), (255, 0, 0), 2)
        
        # Draw region circles & rectangle
        bbox = qrs[0].rect
        cv2.rectangle(frame, (bbox.left, bbox.top), (bbox.left + bbox.width, bbox.top + bbox.height), (0, 255, 0), 2)
        cv2.circle(frame, (cx, cy), in_area, (255, 0, 0), 2)  # Center region (blue)
        cv2.circle(frame, (cx, cy), out_area, (255, 0, 0), 2)  # Middle region (blue)
        
        cv2.putText(frame, f"Error: {math.sqrt(error_x**2 + error_y**2):.2f} px", (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)
        
    else:
        send_local_ned_velocity(0, 0, 0, 1)  # hover in place if no QR detected

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    cv2.imshow("Region-Based QR Tracking", frame)

cap.release()
cv2.destroyAllWindows()

# ORIENT WITH THE QR
if qrs:
    qr_angle = get_qr_angle(qrs[0])
    print(f"QR angle: {qr_angle} degrees")
    current_yaw = vehicle.heading
    desired_yaw = (current_yaw + qr_angle) % 360
    yaw_diff = (desired_yaw - current_yaw + 540) % 360 - 180  # shortest angle difference
    direction = 1 if yaw_diff > 0 else -1
    condition_yaw(abs(yaw_diff), is_relative=1, direction=direction)


# MAKE THE DRONE LAND ON THE QR
print("QR centered, descending to 1 meter...")
target_alt = 1
while vehicle.location.global_relative_frame.alt > target_alt + 0.5: # 0.5m buffer
    send_local_ned_velocity(0, 0, 0.5, 0.5) # Send 0.5 m/s downward velocity
    time.sleep(0.5)

# Ensure it stops and is at least 1m
send_local_ned_velocity(0, 0, 0, 1)
print(f"Reached {vehicle.location.global_relative_frame.alt:.2f} m altitude. Preparing for RTL.")

# RETURN TO ORIGINAL HOME LOCATION
print("Returning to original home location...")
vehicle.mode = VehicleMode("RTL")
time.sleep(2)

# CLEAN UP THE CODE
vehicle.close()
