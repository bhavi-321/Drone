#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import print_function
import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from pyzbar.pyzbar import decode
import time
import math
import argparse
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Quaternion
import tf.transformations as tft

# ---------------------------------------------
# ---------------------------------------------

parser = argparse.ArgumentParser(description='Drone QR Code Follower (Gazebo Camera)')
parser.add_argument('--connect', 
                    help="Vehicle connection target string",  
                    default='127.0.0.1:14550')
parser.add_argument('--topic',
                    help="ROS camera topic (default: /camera_link/image_raw)",
                    default='/camera_link/image_raw')
args = parser.parse_args()

connection_string = args.connect
camera_topic = args.topic

sitl = None
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

bridge = CvBridge()
frame = None

# ---------------------------------------------
# ---------------------------------------------

def arm_and_takeoff(target_altitude):
    print("Arming and taking off...")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    vehicle.simple_takeoff(target_altitude)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {alt:.1f}")
        if alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)
        
# ---------------------------------------------
# ---------------------------------------------

def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
    t0 = math.cos(yaw * 0.5)
    t1 = math.sin(yaw * 0.5)
    t2 = math.cos(roll * 0.5)
    t3 = math.sin(roll * 0.5)
    t4 = math.cos(pitch * 0.5)
    t5 = math.sin(pitch * 0.5)

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5
    return [w, x, y, z]

# ---------------------------------------------
# ---------------------------------------------


def send_attitude_command(roll_angle=0.0, pitch_angle=0.0, yaw_rate=0.0, thrust=0.5, duration=4):
    """
    Send attitude commands continuously for 'duration' seconds.
    Roll, pitch, yaw_rate in degrees.
    """
    roll = math.radians(roll_angle)
    pitch = math.radians(pitch_angle)
    yaw_rate = math.radians(yaw_rate)

    q = to_quaternion(roll, pitch, 0)

    # type_mask = 0b00000100 → ignored body rates, so drone didn't yaw.
    # FIX: use 0b00000000 → all fields active.
    type_mask = 0b00000000

    msg = vehicle.message_factory.set_attitude_target_encode(
        0,  # time_boot_ms
        0, 0,  # target system, component
        type_mask,
        q,
        0, 0, yaw_rate,  # roll_rate, pitch_rate, yaw_rate
        thrust
    )

    start = time.time()
    while time.time() - start < duration:
        vehicle.send_mavlink(msg)
        vehicle.flush()
        time.sleep(0.1)


# ---------------------------------------------
# ---------------------------------------------


def send_attitude_command(roll_angle=0.0, pitch_angle=0.0, yaw_rate=0.0, thrust = 0.5, duration=0.01):
    
    roll = math.radians(roll_angle)
    pitch = math.radians(pitch_angle)
    yaw_rate = math.radians(yaw_rate)
    q = to_quaternion(roll, pitch, 0)
    vehicle.groundspeed = 0.25

    msg = vehicle.message_factory.set_attitude_target_encode(
        0, 0, 0, 0b00000000,  # type mask
        q, 0, 0, yaw_rate, thrust)

    start = time.time()
    while time.time() - start < duration:
        vehicle.send_mavlink(msg)
        vehicle.flush()
        # time.sleep(0.05)
        
# ---------------------------------------------
# ---------------------------------------------


def check_qr(image_path):
    img = cv2.imread(image_path)
    qr_codes = decode(img)
    if qr_codes:
        return qr_codes[0].data.decode('utf-8')
    return None

# ---------------------------------------------
# ---------------------------------------------


def qr_center(qr):
    pts = qr.polygon
    M = cv2.moments(np.array([(pt.x, pt.y) for pt in pts]))
    if M['m00'] == 0:
        return None
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    return (cx, cy)

# ---------------------------------------------
# ---------------------------------------------


def just_go(point, point_lat, point_lon):
    lat_self = vehicle.location.global_relative_frame.lat
    lon_self = vehicle.location.global_relative_frame.lon
    alt_self = vehicle.location.global_relative_frame.alt
    vehicle.simple_goto(point)
    while distance(lat_self, lon_self, point_lat, point_lon) >= 1:
        lat_self = vehicle.location.global_relative_frame.lat
        lon_self = vehicle.location.global_relative_frame.lon
        print(distance(lat_self, lon_self, point_lat, point_lon), alt_self)
        time.sleep(1)
        
# ---------------------------------------------
# ---------------------------------------------


def distance(p1_lat, p1_lon, p2_lat, p2_lon):
    p1_lat = math.radians(p1_lat)
    p2_lat = math.radians(p2_lat)
    p1_lon = math.radians(p1_lon)
    p2_lon = math.radians(p2_lon)
    del_lat = p1_lat - p2_lat
    del_lon = p1_lon - p2_lon
    dist = 2*R * math.asin(math.sqrt((math.sin(del_lat/2))**2 + math.cos(p2_lat) * math.cos(p1_lat) * (math.sin(del_lon/2))**2))
    return dist

# ---------------------------------------------
# ---------------------------------------------


def check_qr(image_path):
    img = cv2.imread(image_path)
    qr_codes = decode(img)
    if qr_codes:
        for qr in qr_codes:
            check_qr_data = qr.data.decode('utf-8')
            # print(qr_data)  
    return check_qr_data

# ---------------------------------------------
# ---------------------------------------------

def image_callback(msg):
    global frame
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
# ---------------------------------------------
# ---------------------------------------------


R = 6371000
alt_self = vehicle.location.global_relative_frame.alt
lat_self = vehicle.location.global_relative_frame.lat
lon_self = vehicle.location.global_relative_frame.lon
lat_home = vehicle.location.global_relative_frame.lat
lon_home = vehicle.location.global_relative_frame.lon
point1 = LocationGlobalRelative(-35.36284230, 149.16515144, 5)
target_qr = check_qr("//home//bhavi//dronekit-python//examples//guided_set_speed_yaw//image_qr_check.png")

bridge = CvBridge()
frame = None

print(f"Subscribing to camera topic: {camera_topic}")
rospy.init_node('qr_follower_camera_node', anonymous=True)
rospy.Subscriber(camera_topic, Image, image_callback)

# Wait for camera feed
print("Waiting for camera feed...")
while frame is None and not rospy.is_shutdown():
    rospy.sleep(0.1)
print("Camera feed received!")

arm_and_takeoff(5)
target_qr = check_qr("//home//bhavi//dronekit-python//examples//guided_set_speed_yaw//image_qr_check.png")

while not rospy.is_shutdown():

    img = frame.copy()
    h, w, _ = img.shape
    frame_center = (w // 2, h // 2)
    for qr in decode(img):
        qr_data = qr.data.decode('utf-8')
        if qr_data != target_qr:
            continue
        cx, cy = qr_center(qr)
        pts = np.array([(pt.x, pt.y) for pt in qr.polygon]).reshape((-1, 1, 2))
        cv2.polylines(img, [pts], True, (0, 255, 0), 2)
        cv2.circle(img, (cx, cy), 5, (0, 0, 255), -1)
        x_error = cx - frame_center[0]
        y_error = frame_center[1] - cy
        if abs(x_error) < 20:
            x_error = 0
        if abs(y_error) < 20:
            y_error = 0
            
        # alpha = 0.7  # 0.0=instant, 1.0=slow
        # x_error = alpha * prev_x_error + (1 - alpha) * x_error if 'prev_x_error' in locals() else x_error
        # y_error = alpha * prev_y_error + (1 - alpha) * y_error if 'prev_y_error' in locals() else y_error
        # prev_x_error, prev_y_error = x_error, y_error
        yaw_rate = np.clip(x_error / 200.0, -2, 2)
        roll_angle = np.clip(x_error / 300.0, -2, 2)
        pitch_angle = np.clip(-y_error / 300.0, -2, 2)
        target_alt = 6.0  # meters (your desired altitude)
        alt = vehicle.location.global_relative_frame.alt
        alt_error = target_alt - alt

        # Altitude PID
        thrust_base = 0.45
        k_alt = 0.02
        thrust = np.clip(thrust_base + k_alt * alt_error, 0.3, 0.6)

        
        
        send_attitude_command(roll_angle, pitch_angle, yaw_rate, thrust=thrust, duration=0.05)

        # send_attitude_command(roll_angle, pitch_angle, yaw_rate, thrust=0.35, duration=0.1)
    cv2.circle(img, frame_center, 5, (0, 0, 255), -1)
    cv2.imshow("Gazebo QR Follower", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    
cv2.destroyAllWindows()
vehicle.close()
if sitl:
    sitl.stop()
