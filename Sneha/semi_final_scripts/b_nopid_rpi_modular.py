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
9) once centered, drone descends and lands on the QR code
10) after landing, drone RTLs to original home location
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
EARTH_RADIUS = 6371
TARGET_LAT = -35.36281062
TARGET_LON = 149.16515042
TARGET_ALT = 5
SPEED = 1
CONNECTION_STRING = '/dev/ttyACM0'

in_area = 10
out_area = 100
b_vel = 0.3
less_vel = 0.15


class DroneController:
    """Controller for drone operations"""
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def arm_and_takeoff(self, target_altitude):
        """Arm the vehicle and fly to target_altitude (m)."""
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
        self.vehicle.simple_takeoff(target_altitude)

        while True:
            alt = self.vehicle.location.global_relative_frame.alt
            print(f" Altitude: {alt} m")
            if alt >= target_altitude * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)

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

    def send_local_ned_velocity(self, velocity_x, velocity_y, velocity_z, duration):
        """
        Move vehicle in direction based on specified velocity vectors.
        velocity_x: velocity in m/s along North direction
        velocity_y: velocity in m/s along East direction
        velocity_z: velocity in m/s along Down direction (positive downward)
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0, 0, 0,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b0000111111000111,
            0, 0, 0,
            velocity_x, velocity_y, velocity_z,
            0, 0, 0,
            0, 0)
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def condition_yaw(self, heading, is_relative, direction, duration=2):
        """
        Send MAV_CMD_CONDITION_YAW message to point vehicle at a specified heading (in degrees).
        heading: yaw in degrees, 0-360
        is_relative: 1 for relative yaw, 0 for absolute angle
        direction: -1 for counter-clockwise, 1 for clockwise, 0 for shortest path
        """
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
        time.sleep(5)
        print("Returning to Launch")
        self.vehicle.mode = VehicleMode("RTL")
        time.sleep(2)


class QRDetector:
    """Handler for QR code detection and processing"""
    def __init__(self):
        self.DETECTED = False

    def initialize_camera(self):
        """Initialize camera capture"""
        cap = cv2.VideoCapture(
            "libcamerasrc ! video/x-raw,width=1280,height=720,framerate=30/1 ! videoconvert ! appsink",
            cv2.CAP_GSTREAMER)
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

    def calculate_region_velocity(self, error_x, error_y):
        """
        Calculate velocity based on error regions.
        Returns (vx, vy, region) velocities in m/s
        
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

    def visualize_region_tracking(self, frame, qr, error_x, error_y):
        """Draw region-based tracking visualization on frame"""
        vertices = qr.polygon
        cx, cy = frame.shape[1]//2, frame.shape[0]//2
        x, y = self.get_qr_center(qr)
        
        cv2.circle(frame, (x, y), 5, (0, 255, 0), -1)
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        cv2.line(frame, (cx, cy), (x, y), (255, 0, 0), 2)
        
        # Draw region circles
        cv2.circle(frame, (cx, cy), in_area, (255, 0, 0), 2)  # Center region
        cv2.circle(frame, (cx, cy), out_area, (255, 0, 0), 2)  # Middle region
        
        cv2.putText(frame, f"Error: {math.sqrt(error_x**2 + error_y**2):.2f} px",
                   (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        return frame


class Mission:
    """Main mission coordinator"""
    def __init__(self, vehicle, drone_controller, qr_detector):
        self.vehicle = vehicle
        self.drone = drone_controller
        self.qr = qr_detector

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
        self.vehicle.mode = VehicleMode("LAND")
        time.sleep(10)
        print("Returning to Launch")
        self.vehicle.mode = VehicleMode("RTL")
        time.sleep(2)
        self.vehicle.close()
        exit()

    def search_for_qr_at_target(self, cap, point1):
        """Search for QR code while approaching target"""
        qrs = None
        
        while not qrs and not self.qr.DETECTED:
            ret, frame = cap.read()
            if not ret:
                print("Could not capture frame.")
                time.sleep(0.1)
                continue

            qrs = decode(frame)
            if qrs:
                print(f"Detected QR code with data: {qrs[0].data.decode('utf-8')}")
                print("QR code detected. Centering above QR code...")
                self.qr.DETECTED = True
                break

            dist = self.drone.get_distance_to_target(point1)
            print(f"distance : {round(dist, 2)} m")
            
            if dist <= 5:
                break

            cv2.imshow("Region-Based QR Tracking", frame)
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
                time.sleep(0.1)
                continue

            qrs = decode(frame)
            if qrs:
                print(f"Detected QR code with data: {qrs[0].data.decode('utf-8')}")
                print("QR code detected. Centering above QR code...")
                self.qr.DETECTED = True
                break

            self.drone.send_local_ned_velocity(SPEED, 0, 0, 1)
            dist = self.drone.get_distance_to_target(point1)
            print(f"distance : {round(dist, 2)} m")

            if dist > 5:
                cv2.destroyAllWindows()
                print("Could not find QR code, landing at target location and returning home...")
                self.vehicle.simple_goto(point1)
                time.sleep(5)
                self.drone.descend_to_altitude(3)
                self.drone.land_and_rtl()
                self.vehicle.close()
                exit()
            else:
                cv2.imshow("Region-Based QR Tracking", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue
        
        return qrs

    def center_above_qr_region_based(self, cap):
        """Use region-based control to center drone above QR code"""
        self.vehicle.mode = VehicleMode("GUIDED")
        self.drone.condition_yaw(self.vehicle.heading, is_relative=0, direction=0)

        print("Starting region-based centering...")
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Could not capture frame.")
                time.sleep(0.1)
                continue

            qrs = decode(frame)
            if qrs:
                cx, cy = frame.shape[1]//2, frame.shape[0]//2
                x, y = self.qr.get_qr_center(qrs[0])
                error_x = x - cx
                error_y = y - cy

                # Calculate velocities based on regions
                vx, vy, region = self.qr.calculate_region_velocity(error_x, error_y)
                
                # Check if centered
                if region == "CENTERED":
                    print("QR code centered!")
                    self.drone.send_local_ned_velocity(0, 0, 0, 1)
                    break
                
                # Invert vy direction to move toward QR
                vy = -vy
                self.drone.send_local_ned_velocity(vy, vx, 0, 1)
                
                print(f"Error: {math.sqrt(error_x**2 + error_y**2):.2f} | vx: {vx:.2f}, vy: {vy:.2f}")
                
                frame = self.qr.visualize_region_tracking(frame, qrs[0], error_x, error_y)
            else:
                self.drone.send_local_ned_velocity(0, 0, 0, 1)  # hover in place if no QR detected

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            cv2.imshow("Region-Based QR Tracking", frame)

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
    vehicle = connect(args.connect, wait_ready=True, baud=921600)

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
    print("Searching for QR code...")

    # Initialize camera
    cap = qr_detector.initialize_camera()

    if not cap.isOpened():
        mission.execute_no_camera_mission(point1)
    else:
        print("Searching for QR code...")

    # Search for QR at target
    qrs = mission.search_for_qr_at_target(cap, point1)

    # Search for QR moving forward
    if not qrs:
        qrs = mission.search_for_qr_moving_forward(cap, point1)

    cv2.destroyAllWindows()

    # Center above QR using region-based control
    qrs = mission.center_above_qr_region_based(cap)

    cap.release()
    cv2.destroyAllWindows()

    # Align with QR orientation
    mission.align_with_qr(qrs)

    # Land on QR
    mission.land_on_qr()

    # Drop payload (commented out)
    # print("Servos actuated")
    # drone.set_servo(6, 1100)
    # time.sleep(5)

    # Return to home
    print("Returning to original home location...")
    vehicle.mode = VehicleMode("RTL")
    time.sleep(2)

    # servo close?

    # Clean up
    vehicle.close()


if __name__ == "__main__":
    main()