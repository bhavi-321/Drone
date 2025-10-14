from __future__ import print_function
import time
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import cv2
import numpy as np
from pyzbar.pyzbar import decode

FACTOR = 1.113195e5

# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.",default = '127.0.0.1:14550')
args = parser.parse_args()

connection_string = args.connect
sitl = None


# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

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
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

def condition_yaw(heading, relative=False):
    from pymavlink import mavutil
    is_relative = 1 if relative else 0
    msg = vehicle.message_factory.command_long_encode(
        0, 0,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading,
        0,
        1,
        is_relative,
        0, 0, 0
    )
    vehicle.send_mavlink(msg)

def follow_qr():
    expected_qr = cv2.imread("/home/rishita/Downloads/frame.png")
    expected_qr_codes = decode(expected_qr)
    if expected_qr_codes:
        expected_qr_data = expected_qr_codes[0].data.decode('utf-8')
        print(f"Expected QR data: {expected_qr_data}")
    else:
        print("No QR code found in frame.png!")
        expected_qr_data = None

    cap = cv2.VideoCapture(0)
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_center_x = frame_width // 2 if frame_width > 0 else 320  # fallback

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        qr_code = decode(frame)
        target_found = False

        for qr in qr_code:
            qr_data = qr.data.decode('utf-8')
            if expected_qr_data and qr_data == expected_qr_data:
                x, y, w, h = qr.rect
                x, y, w, h = qr.rect
                cx = x + w // 2
                cy = y + h // 2
                target_found = True

                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(frame, qr_data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

                error_x = cx - frame_center_x

                if abs(error_x) > 40:
                    if error_x > 0:
                        print("QR right, yaw right")
                        condition_yaw(10, relative=True)
                    else:
                        print("QR left, yaw left")
                        condition_yaw(-10, relative=True)
                else:
                    print("QR centered, move forward")
                    current_location = vehicle.location.global_relative_frame
                    target_location = LocationGlobalRelative(
                        current_location.lat + 0.00001, current_location.lon, current_location.alt)
                    vehicle.simple_goto(target_location)
                break
            else:
                print(f"QR detected but does not match expected: {qr_data}")

        cv2.imshow("QR Detection", frame)
        if cv2.waitKey(1) & 0xFF == 32:  # Press space to exit
            break

        if not target_found:
            print("No matching QR detected. Hovering...")


        cv2.imshow("QR Detection", frame)
        if cv2.waitKey(1) & 0xFF == 32:  # Press space to exit
            break

        if not target_found:
            print("No QR detected. Hovering...")

    cap.release()
    cv2.destroyAllWindows()

arm_and_takeoff(10)
time.sleep(5)
vehicle.airspeed = 3
target_location = LocationGlobalRelative(-35.361354, 149.16526529, 20)
vehicle.simple_goto(target_location)
follow_qr()
print("Following QR code...")

