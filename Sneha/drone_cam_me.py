# IMPORT REQUIRED LIBRARIES
import time
import math
import argparse
import cv2
from pyzbar.pyzbar import decode
from PIL import Image
import numpy as np
from dronekit import connect, VehicleMode
from pymavlink import mavutil


# DEFINE CONSTANTS
DETECTED = False


# CONNECT THE VEHICLE
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
        mavutil.mavlink.MAV_FRAME_BODY_NED,    # frame
        0b0000111111000111,                    # type_mask (only speeds enabled)
        0, 0, 0,                               # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,    # x, y, z velocity in m/s
        0, 0, 0,                               # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)                                  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    for i in range(duration):
        vehicle.send_mavlink(msg)
        time.sleep(0.1)


class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.sum_error = 0

    def pid_control(self, error, dt):
        error = -error
        self.sum_error += error * dt
        output = (self.kp * error) + (self.ki * self.sum_error) + (self.kd * (error - self.prev_error) / dt)
        self.prev_error = error

        return output

pid_x = PID(kp=0.0015, ki=0.0, kd=0.002)
pid_y = PID(kp=0.0015, ki=0.0, kd=0.002)


# EMBED THE QR TO BE DETECTED
ref_image_path = "/home/sneha/Pictures/Screenshot from 2025-10-07 01-56-01.png"
ref_image = Image.open(ref_image_path)
decoded_objects = decode(ref_image)
data = decoded_objects[0].data.decode('utf-8')

# ARM AND TAKEOFF TO A SPECIFIED ALTITUDE
arm_and_takeoff(10)


cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Could not detect camera.")
    vehicle.mode = VehicleMode("RTL")
    vehicle.close()
    exit()

then = time.time()

while True:
    ret, frame = cap.read()

    if not ret:
        print("Could not capture frame.")
        break

    qrs = decode(frame)
    
    if qrs:
        if qrs[0].data.decode('utf-8') == data:
            vertices = qrs[0].polygon

            cx , cy = frame.shape[1]//2, frame.shape[0]//2         # centre of frame, x is height, y is width
            x = int((vertices[0][0] + vertices[1][0]) // 2)  # centre of detected qr
            y = int((vertices[0][1] + vertices[3][1]) // 2)  # centre of detected qr
            error_x = x - cx
            error_y = y - cy

            if math.sqrt(error_x**2 + error_y**2) < 50:
                break

            now = time.time()
            dt = now - then
            then = now

            vx = -pid_x.pid_control(error_x, dt)    # change in velocity left/right
            vy = pid_y.pid_control(error_y, dt)   # change in velocity forward/backward
            vz = 0

            vx = np.clip(vx, -1, 1)
            vy = np.clip(vy, -1, 1)

            send_local_ned_velocity(vy, vx, vz, 1)

            # visualise

            cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            cv2.line(frame, (cx, cy), (x, y), (255, 0, 0), 2)
            cv2.rectangle(frame, (int(vertices[0][0]), int(vertices[0][1])), (int(vertices[2][0]), int(vertices[2][1])), (0, 255, 0), 2)
            

        else:
            send_local_ned_velocity(0, 0, 0, 1)  # hover in place if no QR detected

    else:
        send_local_ned_velocity(0, 0, 0, 1)  # hover in place if no QR detected
    

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    cv2.imshow("PID QR Tracking", frame)


print("Returning to Launch...")
vehicle.mode = VehicleMode("RTL")
vehicle.close()
cap.release()
cv2.destroyAllWindows()