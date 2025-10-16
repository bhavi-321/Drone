#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy as np
import math
import time
import argparse
import threading
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from pyzbar.pyzbar import decode

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

print("Disabling arming checks...")
vehicle.parameters['ARMING_CHECK'] = 0
vehicle.parameters['THR_FAILSAFE'] = 0

R = 6371000  # Earth radius


def arm_and_takeoff(aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.flush()
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def distance(p1_lat, p1_lon, p2_lat, p2_lon):
    p1_lat, p2_lat = map(math.radians, [p1_lat, p2_lat])
    p1_lon, p2_lon = map(math.radians, [p1_lon, p2_lon])
    del_lat, del_lon = p1_lat - p2_lat, p1_lon - p2_lon
    dist = 2 * R * math.asin(math.sqrt(
        (math.sin(del_lat / 2)) ** 2 +
        math.cos(p1_lat) * math.cos(p2_lat) * (math.sin(del_lon / 2)) ** 2))
    return dist


def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    t0 = math.cos(yaw * 0.5)
    t1 = math.sin(yaw * 0.5)
    t2 = math.cos(roll * 0.5)
    t3 = math.sin(roll * 0.5)
    t4 = math.cos(pitch * 0.5)
    t5 = math.sin(pitch * 0.5)
    return [t0 * t2 * t4 + t1 * t3 * t5,
            t0 * t3 * t4 - t1 * t2 * t5,
            t0 * t2 * t5 + t1 * t3 * t4,
            t1 * t2 * t4 - t0 * t3 * t5]


attitude_control = {"roll": 0, "pitch": 0, "yaw_rate": 0, "thrust": 0.5}
running = False


def send_attitude_stream():
    while running:
        roll = math.radians(attitude_control["roll"])
        pitch = math.radians(attitude_control["pitch"])
        yaw_rate = math.radians(attitude_control["yaw_rate"])
        thrust = attitude_control["thrust"]

        q = to_quaternion(roll, pitch, 0)
        msg = vehicle.message_factory.set_attitude_target_encode(
            0, 0, 0,
            0b00000000,
            q,
            0, 0, yaw_rate,
            thrust
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()
        time.sleep(0.05)


def get_qr_angle(qr):
    pts = np.array([(pt.x, pt.y) for pt in qr.polygon])
    dx, dy = pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]
    return np.degrees(np.arctan2(dy, dx))


def dist_point(qrx, qry, framex, framey):
    distance = (((qrx - framex) ** 2) + ((qry - framey) ** 2)) ** 0.5
    return abs(distance)


def dist_2_angle(x, y, center):
    return (center[0] - x) / center[0] * 90


def check_qr(img_path):
    img = cv2.imread(img_path)
    codes = decode(img)
    return codes[0].data.decode('utf-8') if codes else None


arm_and_takeoff(10)
vehicle.airspeed = 1

target_point = LocationGlobalRelative(-35.36304334, 149.16518389, 10)
print("Flying to target coordinate...")
vehicle.simple_goto(target_point)

cap = None
qr_mode_started = False
target_qr = check_qr("/home/bhavi/dronekit-python/examples/guided_set_speed_yaw/check_qr.jpg")

while True:
    lat = vehicle.location.global_relative_frame.lat
    lon = vehicle.location.global_relative_frame.lon
    alt = vehicle.location.global_relative_frame.alt
    dist = distance(lat, lon, target_point.lat, target_point.lon)
    print(f"Distance: {dist:.2f} m  Alt: {alt:.2f} m")

    if not qr_mode_started and dist <= 10:
        vehicle.airspeed = 0.5
        qr_mode_started = True
        running = True
        stream_thread = threading.Thread(target=send_attitude_stream)
        stream_thread.daemon = True
        stream_thread.start()
        cap = cv2.VideoCapture(0)

    if qr_mode_started:
        ret, frame = cap.read()
        if not ret:
            continue
        h, w, _ = frame.shape
        center = (w // 2, h // 2)

        qr_codes = decode(frame)
        for qr in qr_codes:
            qr_data = qr.data.decode('utf-8')
            M = cv2.moments(np.array([(pt.x, pt.y) for pt in qr.polygon]))
            if M['m00'] == 0:
                continue
            qr_x, qr_y = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
            cv2.circle(frame, (qr_x, qr_y), 3, (0, 0, 255), 2)

            if qr_data == target_qr:
                angle_err = dist_2_angle(qr_x, qr_y, center)
                tilt_err = get_qr_angle(qr)
                vert_err = center[1] - qr_y
                vehicle.airspeed = 0.5

                attitude_control["roll"] = 1 if angle_err > 10 else -1 if angle_err < -10 else 0
                attitude_control["yaw_rate"] = 10 if tilt_err > 5 else -10 if tilt_err < -5 else 0
                attitude_control["pitch"] = -1 if vert_err > 20 else 1 if vert_err < -20 else 0
                attitude_control["thrust"] = 0.52
                vehicle.airspeed = 0.5

                if dist_point(qr_x, qr_y, center[0], center[1]) <= 50:
                    attitude_control["roll"] = 1 if angle_err > 10 else -1 if angle_err < -10 else 0
                    attitude_control["yaw_rate"] = 10 if tilt_err > 5 else -10 if tilt_err < -5 else 0
                    attitude_control["pitch"] = -1 if vert_err > 20 else 1 if vert_err < -20 else 0
                    attitude_control["thrust"] = 0.3
                    vehicle.airspeed = 0.5
                else:
                    continue

                if vehicle.location.global_relative_frame.alt <= 2:
                    print("Landing initiated...")
                    vehicle.mode = VehicleMode("LAND")

                    # aa main che ke wait until landed and disarmed before any RTL or close
                    while True:
                        alt = vehicle.location.global_relative_frame.alt
                        print(f"Waiting for complete landing... Altitude: {alt:.2f} m")
                        if not vehicle.armed or (alt is not None and alt <= 0.2):
                            print("Landed successfully and disarmed.")
                            break
                        time.sleep(1)

                    # stop attitude streaming.....
                    running = False
                    time.sleep(0.5)

                    if cap:
                        cap.release()
                    cv2.destroyAllWindows()

                    # aa main che ke only RTL after confirmed landing
                    print("Returning to Launch (RTL)...")
                    vehicle.mode = VehicleMode("RTL")

                    break  # Qr vado loop break
            else:
                time.sleep(10)
                attitude_control["roll"] = attitude_control["pitch"] = attitude_control["yaw_rate"] = 0
                print("No QR found, RTL")
                print("Returning to Launch")
                vehicle.mode = VehicleMode("RTL")
        cv2.circle(frame, (center), 5, (0, 255, 0), -1)
        cv2.imshow("QR Follow", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

running = False
time.sleep(0.2)
if cap:
    cap.release()
cv2.destroyAllWindows()
vehicle.close()
print("Closed cleanly.")