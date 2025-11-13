#!/usr/bin/env python3

'''
1) drone takes off to a specified altitude
2) camera feed initialises to read QR code
3) if it doesn't detect camera, drone goes to the target location, drops the payload and RTLs to home
4) if it detects camera, drone goes to the target location
5) drone searches for a specific QR code
6) if QR code is not found, drone moves 5m ahead to search for QR code
7) if QR code is not found, drone lands at target location and RTLs to home
8) if QR code is found, drone uses PID to center itself above the QR code
9) once centered, drone descends and lands on the QR code
10) it drops the payload  #abtak nhi kiya
11) after landing, drone takes off to an altitude, after which the servo motors close (ya jo bhi tha idr)
12) drone RTLs to original home location
'''

import time
import math
import argparse
import cv2
from pyzbar.pyzbar import decode
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil


# CONSTANTS
FACTOR = 1.113195e5
TARGET_LAT = -35.36281062
TARGET_LON = 149.16515042
TARGET_ALT = 5
SPEED = 1
CONNECTION_STRING = '127.0.0.1:14550'
VIDEO_SOURCE = 0

# line 161 remove 2nd param if not using gstreamer
# line 366 remove baud rate param if using tcp/udp


class PID:
    """PID Controller for drone positioning"""
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.sum_error = 0

    def pid_control(self, error, dt):
        error = -error
        self.sum_error += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0
        output = (self.kp * error) + (self.ki * self.sum_error) + (self.kd * derivative)
        self.prev_error = error
        return output


class DroneController:
    """Controller for drone operations"""
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def arm_and_takeoff(self, target_alitude):
        """Arm the vehicle and fly to TARGET_ALTitude (m)."""
        while not self.vehicle.is_armable:
            print(" Waiting for vehicle to initialise...")
            time.sleep(1)

        print("Arming motors...")
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not self.vehicle.armed:
            print(" Waiting for arming...")
            time.sleep(1)
        
        print("Taking off!")
        self.vehicle.simple_takeoff(target_alitude)

        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            print(f" Altitude: {alt} m")
            if alt >= target_alitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

    def send_local_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        """Send velocity commands in body NED frame"""
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111,
            0, 0, 0,
            velocity_x, velocity_y, velocity_z,
            0, 0, 0,
            0, 0)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def condition_yaw(self, heading, is_relative, direction, duration=2):
        """Set drone yaw angle"""
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            heading,
            0,
            direction,
            is_relative,
            0, 0, 0)
        
        for i in range(duration):
            self.vehicle.send_mavlink(msg)
            time.sleep(0.1)
        
        self.vehicle.flush()

    def set_servo(self, servo_number, pwm_value):
        """Control servo motors"""
        pwm_value_int = int(pwm_value)
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,
            mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
            0,
            servo_number,
            pwm_value_int,
            0, 0, 0, 0, 0)
        self.vehicle.send_mavlink(msg)

    def get_distance_to_target(self, target_location):
        """Calculate distance to target location"""
        current_location = self.vehicle.location.global_relative_frame
        dist = FACTOR * ((current_location.lat - target_location.lat)**2 + 
                        (current_location.lon - target_location.lon)**2)**0.5
        return dist

    def descend_to_altitude(self, target_alt):
        """Descend to specified altitude"""
        while self.vehicle.location.global_relative_frame.alt > target_alt:
            self.send_local_ned_velocity(0, 0, 1, 1)

    def land_and_rtl(self):
        """Land and return to launch"""
        print("Landing...")
        self.vehicle.mode = VehicleMode("LAND")
        time.sleep(10)
        print("Returning to Launch")
        self.vehicle.mode = VehicleMode("RTL")
        time.sleep(2)


class QRDetector:
    """Handler for QR code detection and processing"""
    def __init__(self):
        self.DETECTED = False

    def initialize_camera(self):
        """Initialize camera capture"""
        cap = cv2.VideoCapture(VIDEO_SOURCE)  #remove the 2nd param if not using gstreamer
        return cap

    def detect_qr(self, frame):
        """Detect QR codes in frame"""
        qrs = decode(frame)
        return qrs

    def get_qr_center(self, qr):
        """Calculate center of QR code"""
        vertices = qr.polygon
        x = int((vertices[0][0] + vertices[1][0]) // 2)
        y = int((vertices[0][1] + vertices[3][1]) // 2)
        return x, y

    def get_qr_angle(self, qr):
        """Calculate QR code orientation angle"""
        pts = np.array([(pt.x, pt.y) for pt in qr.polygon])
        dx, dy = pts[1][0] - pts[0][0], pts[1][1] - pts[0][1]
        return np.degrees(np.arctan2(dy, dx))

    def visualize_tracking(self, frame, qr, error_x, error_y):
        """Draw tracking visualization on frame"""
        vertices = qr.polygon
        cx, cy = frame.shape[1]//2, frame.shape[0]//2
        x, y = self.get_qr_center(qr)
        
        cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        cv2.line(frame, (cx, cy), (x, y), (255, 0, 0), 2)
        cv2.rectangle(frame, (int(vertices[0][0]), int(vertices[0][1])),
                     (int(vertices[2][0]), int(vertices[2][1])), (0, 255, 0), 2)
        cv2.putText(frame, f"Error: {math.sqrt(error_x**2 + error_y**2)}",
                   (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        return frame


class Mission:
    """Main mission coordinator"""
    def __init__(self, vehicle, drone_controller, qr_detector):
        self.vehicle = vehicle
        self.drone = drone_controller
        self.qr = qr_detector
        self.pid_x = PID(kp=0.003, ki=0.0, kd=0.004)
        self.pid_y = PID(kp=0.003, ki=0.0, kd=0.004)

    def execute_no_camera_mission(self, point1):
        """Execute mission when camera is not detected"""
        print("Could not detect camera.")
        print("Going to target location without QR detection...")

        while True:
            dist = self.drone.get_distance_to_target(point1)
            print(f"distance to target location: {round(dist, 2)} m")
            if dist <= 1:
                print("Reached target location.")
                break
            time.sleep(1)
        
        print("Landing at target location...")
        self.drone.descend_to_altitude(3)
        self.drone.land_and_rtl()
        self.vehicle.close()
        exit()

    def search_for_qr_at_target(self, cap, point1):
        """Search for QR code while moving to target"""
        qrs = None
        
        while not qrs and not self.qr.DETECTED:
            ret, frame = cap.read()
            if not ret:
                print("Could not capture frame.")
                continue

            qrs = decode(frame)
            if qrs:
                print(f"Detected QR code with data: {qrs[0].data.decode('utf-8')}")
                print("QR code detected. Centering above QR code...")
                self.qr.DETECTED = True
                break

            dist = self.drone.get_distance_to_target(point1)
            print(f"distance to point 1: {round(dist, 2)} m")
            
            if dist <= 5:
                break

            cv2.imshow("PID QR Tracking", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        return qrs

    def search_for_qr_moving_forward(self, cap, point1):
        """Search for QR code by moving forward"""
        qrs = None
        
        while not qrs and not self.qr.DETECTED:
            ret, frame = cap.read()
            if not ret:
                print("Could not capture frame.")
                continue

            qrs = decode(frame)
            if qrs:
                print(f"Detected QR code with data: {qrs[0].data.decode('utf-8')}")
                print("QR code detected. Centering above QR code...")
                self.qr.DETECTED = True
                break

            self.drone.send_local_ned_velocity(SPEED, 0, 0, 1)
            dist = self.drone.get_distance_to_target(point1)
            print(f"distance to point 1: {round(dist, 2)} m")

            if dist > 5:
                cap.release()
                cv2.destroyAllWindows()
                print("Could not find QR code, landing at target location and returning home...")
                self.vehicle.simple_goto(point1)
                time.sleep(5)
                self.drone.descend_to_altitude(3)
                self.drone.land_and_rtl()
                self.vehicle.close()
                exit()

            cv2.imshow("PID QR Tracking", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        
        return qrs

    def center_above_qr(self, cap):
        """Use PID to center drone above QR code"""
        then = time.time()
        self.vehicle.mode = VehicleMode("GUIDED")
        self.drone.condition_yaw(self.vehicle.heading, is_relative=0, direction=0)

        while True:
            ret, frame = cap.read()
            if not ret:
                print("Could not capture frame.")
                continue

            qrs = decode(frame)
            if qrs:
                cx, cy = frame.shape[1]//2, frame.shape[0]//2
                x, y = self.qr.get_qr_center(qrs[0])
                error_x = x - cx
                error_y = y - cy

                if math.sqrt(error_x**2 + error_y**2) < 50:
                    break

                now = time.time()
                dt = now - then
                then = now

                vx = -self.pid_x.pid_control(error_x, dt)
                vy = self.pid_y.pid_control(error_y, dt)
                vz = 0

                vx = np.clip(vx, -1, 1)
                vy = np.clip(vy, -1, 1)

                self.drone.send_local_ned_velocity(vy, vx, vz, 1)
                frame = self.qr.visualize_tracking(frame, qrs[0], error_x, error_y)
            else:
                self.drone.send_local_ned_velocity(0, 0, 0, 1)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            cv2.imshow("PID QR Tracking", frame)

        return qrs

    def align_with_qr(self, qrs):
        """Align drone orientation with QR code"""
        qr_angle = self.qr.get_qr_angle(qrs[0])
        print(f"QR angle: {qr_angle} degrees")
        current_yaw = self.vehicle.heading
        desired_yaw = (current_yaw + qr_angle) % 360
        yaw_diff = (desired_yaw - current_yaw + 540) % 360 - 180
        direction = 1 if yaw_diff > 0 else -1
        self.drone.condition_yaw(abs(yaw_diff), is_relative=1, direction=direction)

    def land_on_qr(self):
        """Descend and land on QR code"""
        print("QR centered, descending...")
        self.drone.descend_to_altitude(3)
        self.vehicle.mode = VehicleMode("LAND")
        time.sleep(5)


def main():
    # Parse arguments
    parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
    parser.add_argument('--connect',
                        help="Vehicle connection target string.",
                        default=CONNECTION_STRING)
    args = parser.parse_args()

    # Connect to vehicle
    print('Connecting to vehicle on: %s' % args.connect)
    vehicle = connect(args.connect, wait_ready=True)    # dont keep baud if using tcp/udp

    # Initialize components
    drone = DroneController(vehicle)
    qr_detector = QRDetector()
    mission = Mission(vehicle, drone, qr_detector)

    # Arm and takeoff
    drone.arm_and_takeoff(TARGET_ALT)
    vehicle.groundspeed = SPEED

    # Go to target location
    point1 = LocationGlobalRelative(TARGET_LAT, TARGET_LON, TARGET_ALT)
    vehicle.simple_goto(point1)

    # Initialize camera
    cap = qr_detector.initialize_camera()

    if not cap.isOpened():
        mission.execute_no_camera_mission(point1)

    print("Searching for QR code...")

    # Search for QR at target
    qrs = mission.search_for_qr_at_target(cap, point1)

    # Search for QR moving forward
    if not qrs:
        qrs = mission.search_for_qr_moving_forward(cap, point1)

    cv2.destroyAllWindows()

    # Center above QR using PID
    qrs = mission.center_above_qr(cap)

    cap.release()
    cv2.destroyAllWindows()

    # Align with QR orientation
    mission.align_with_qr(qrs)

    # Land on QR
    mission.land_on_qr()

    # Drop payload
    print("Servos actuated. Dropping the payload...")
    drone.set_servo(6, 1100)
    time.sleep(5)

    # Return to home
    print("Returning to original home location...")
    vehicle.mode = VehicleMode("RTL")
    time.sleep(2)

    # Clean up
    vehicle.close()


if __name__ == "__main__":
    main()