from __future__ import print_function
from pyzbar.pyzbar import decode
import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import time
import math

import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.", default='127.0.0.1:14550')
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

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(2)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

arm_and_takeoff(10)


def condition_yaw(heading, is_relative, direction):
    """
    Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).

    This method sets an absolute heading by default, but you can set the `relative` parameter
    to `True` to set yaw relative to the current yaw heading.

    """
    # create the CONDITION_YAW command using command_long_encode()
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        direction,  # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    # send command to vehicle
    vehicle.send_mavlink(msg)
    
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and
    for the specified duration.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, 0, 0, # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z, # x, y, z velocity in m/s
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 
    
    # send command to vehicle on 1 Hz cycle
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
    
def get_qr_data(input_frame):
    try:
        return decode(input_frame)
    except:
        return []

def check_qr(img_path):
    img = cv2.imread(img_path)
    qr_codes = decode(img)
    if qr_codes:
        for qr in qr_codes:
            check_qr_data = qr.data.decode('utf-8')
            # print(qr_data)
    return check_qr_data

target_qr = check_qr("qr.jpg")

video = cv2.VideoCapture(0)
if not video.isOpened():
    print("Error: Could not open video.")
    vehicle.mode = VehicleMode("RTL")
    vehicle.close()
    exit()


while True:
    ret, in_fr = video.read()
    h, w, channel = in_fr.shape
    fr_center = (int(w/2), int(h/2))

    if not ret:
        print("Error: Could not read frame.")
        break
    
    qro = get_qr_data(in_fr)
    if len(qro) != 0:
        for obj in qro:
            text = obj.data.decode('utf-8')
            pts = np.array([obj.polygon], np.int32)
            pts = pts.reshape((4, 1, 2))
            cv2.polylines(in_fr, [pts], True, (255, 100, 5), 2)


            M = cv2.moments(pts)
            if M['m00'] != 0:
                qr_x = int(M['m10'] / M['m00'])
                qr_y = int(M['m01'] / M['m00'])
                cv2.circle(in_fr, (qr_x, qr_y), int(1), (0, 255, 0), 2)

            fx , fy = in_fr.shape[1]//2, in_fr.shape[0]//2

            error_x = qr_x - fx                       # calculate error in x direction
            error_y = qr_y - fy                       # calculate error in y direction

            if text == target_qr:
                print("Target QR detected")
                if abs(error_x) > 30:   # pixel threshold; tune for your camera
                    if error_x < 0:
                        print("Move left (roll left)")
                        send_ned_velocity(0, -0.3, 0, 1)   # move left (negative Y in NED)
                        # optional: small yaw to face QR
                        condition_yaw(2, is_relative=1, direction=-1)
                    else:
                        print("Move right (roll right)")
                        send_ned_velocity(0, 0.3, 0, 1)    # move right (positive Y in NED)
                        condition_yaw(2, is_relative=1, direction=1)

        
                if abs(error_y) > 30:
                # if the error is significant, adjust altitude
                    if error_y < 0:
                        print("Moving up...")
                        send_ned_velocity(0, 0, -0.2, 1)
                    
                    else:
                        print("Moving down...")
                        send_ned_velocity(0, 0, 0.2, 1)

                cv2.putText(in_fr, text, (50, 50), cv2.FONT_HERSHEY_PLAIN,1.5,(255,100,5),2)
            else:
                print("QR not matched")
                inc = "QR not matched"
                cv2.putText(in_fr, inc, (50, 50), cv2.FONT_HERSHEY_PLAIN,1.5,(255,100,5),2)


    cv2.imshow("QR follow",in_fr)

    if cv2.waitKey(1) & 0xFF == ord('q'):                                            
        break

# exiting the loop
print("Returning to Launch...")
vehicle.mode = VehicleMode("RTL")

video.release()
cv2.destroyAllWindows()
vehicle.close()

if sitl:
    sitl.stop()
