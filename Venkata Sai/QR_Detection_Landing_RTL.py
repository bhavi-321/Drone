#!/usr/bin/env python3
# -*- coding: utf-8 -*-


"""
1) Drone flies to a height of 10m
2) Moves forward by 45m
3) Drone's camera is opened to search for a specific QR code
4) If specific QR code is found:
    a) Drone aligns itself to be centered over the QR code
    b) Drone lands
5) If specific QR code is not found:
    a) Drone moves forward till 50th metre followed by moving back 2.5m, right and left by 5m till specific QR code is found
    b) If found during search, aligns and lands
    c) If still not found, drone makes RTL
"""

# --- Importing necessary libraries ---
import time  # used for delays and timing operations.
import math  # provides mathematical functions like sine, cosine, etc., for GPS calculations.
import threading  # allows running the camera scanner in a separate thread to not block flight control.
import cv2  # openCV library for all computer vision tasks (camera feed, image processing).
from pyzbar.pyzbar import decode  # library specifically for finding and decoding QR codes in images.
from dronekit import connect, VehicleMode, LocationGlobalRelative  # core library for connecting to and controlling the drone.
from pymavlink import mavutil  # used for creating low-level MAVLink commands for precise movements.

# ---------- Configuration ----------
CONNECTION_STRING = "127.0.0.1:14550"  # Connection string for the vehicle (*****update if needed).
TAKEOFF_ALT = 10.0  # Target altitude in meters for the initial takeoff.
TARGET_DISTANCE = 50.0  # Distance in meters to fly forward searching for the QR code.
CAMERA_START_DISTANCE = 45.0  # Distance in meters from home to start the camera.
CRUISE_SPEED = 2.0  # Default ground speed in meters/second for `simple_goto` commands.
CANONICAL_QR_PATH = "/home/sai/Phoenix/codes/QR.png"  # File path to the reference QR code image.
ALIGNMENT_PIXEL_TOL = 30  # Acceptable error margin in pixels for QR code alignment.
ALIGNMENT_ITER_LIMIT = 10  # Safety limit for alignment correction steps.
SIDE_CHECK_TIMEOUT = 5  # Duration in seconds to hover and scan at each point during the fallback search.
CONFIRM_HOVER_ALT = 0.2  # Requested low altitude in meters for the drone to hover at to "confirm" the QR location.

# ---------- Connect to vehicle ----------
print("Connecting to vehicle on %s" % CONNECTION_STRING)
vehicle = connect(CONNECTION_STRING, wait_ready=True)   # Connects to the vehicle and waits until it's ready.

# ---------- Utility functions ----------
def get_location_meters_metres(orig, dNorth, dEast):
    earth_radius = 6378137.0  # Set Earth's radius in meters.
    dLat = dNorth / earth_radius    # Calculate the change in latitude based on the North offset.
    dLon = dEast / (earth_radius * math.cos(math.pi * orig.lat / 180))  # Calculate the change in longitude based on the East offset
    newlat = orig.lat + (dLat * 180 / math.pi)  # Calculate the new latitude in degrees.
    newlon = orig.lon + (dLon * 180 / math.pi)  # Calculate the new longitude in degrees.
    return LocationGlobalRelative(newlat, newlon, orig.alt) # Return a new DroneKit location object with the calculated coordinates and original altitude.

def distance_between_locations_m(a, b):
    R = 6371000.0  # Earth's radius in meters.
    lat1, lat2 = math.radians(a.lat), math.radians(b.lat)    # Convert latitudes from degrees to radians for the formula.
    dlat = lat2 - lat1
    dlon = math.radians(b.lon - a.lon)
    val = math.sin(dlat/2)**2 + math.cos(lat1)*math.cos(lat2)*math.sin(dlon/2)**2   # Apply the Haversine formula to find the distance.
    return 2 * R * math.asin(math.sqrt(val))     # Return the final distance in meters.

def simple_goto_location(target_loc, groundspeed=CRUISE_SPEED):
    """Commands the vehicle to fly to a target GPS location."""
    vehicle.airspeed = groundspeed  # Set the target ground speed for the movement.
    vehicle.simple_goto(target_loc)  # Issue the high-level command to fly to the target location.

def send_ned_velocity(north, east, down, duration):
    """
    Sends a velocity command to the drone for precise, short movements.
    'north', 'east', 'down' are speeds in m/s. Down is positive.
    """
    # Create a MAVLink message to control the drone's velocity.
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,  # time_boot_ms (not used)
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # The frame of reference is the drone's body itself.
        0b0000111111000111,  # A bitmask to specify that we are using velocity components, ignoring position.
        0, 0, 0,  # x, y, z position (not used)
        north, east, down,  # vx, vy, vz velocity in m/s.
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0)  # yaw, yaw_rate (not used)
    
    # Send this command repeatedly for the specified duration.
    end = time.time() + duration
    while time.time() < end:
        # Send the MAVLink message to the vehicle.
        vehicle.send_mavlink(msg)
        # Wait for a short moment before sending the next message.
        time.sleep(0.1)

def decode_image_payload(image_path):
    """Reads a reference QR code image and returns its data."""
    img = cv2.imread(image_path)    # reads the image file from the specified path.
    # If the image could not be loaded, raise an error.
    if img is None: raise FileNotFoundError(f"QR image is not found at {image_path}")
    # using the pyzbar library to find and decode QR codes in the image.
    codes = decode(img)
    # If no QR codes were found, raise an error.
    if not codes: raise ValueError("No QR found in canonical image")
    # Return the data from the first found QR code, decoded as a UTF-8 string.
    return codes[0].data.decode('utf-8')

# ---------- Scanner Thread ----------
class QRScanner(threading.Thread):
    """
    A separate thread to handle camera capture and QR code scanning
    so it doesn't block the main drone control loop.
    """
    def __init__(self, device_index=0):
        # Initialize the parent Thread class.
        super().__init__()
        self.device_index = device_index    # stores the camera device index (e.g., 0 for /dev/video0).
        self.capture = None     # Placeholder for the OpenCV VideoCapture object.
        self.running = threading.Event()    # A threading event to control the main loop of this thread.
        self.detected = threading.Event()   # A threading event to signal to the main thread that the QR code has been found.
        self.detected_frame = None   # A placeholder to store the video frame in which the QR code was detected.
        self.detected_box = None   # A placeholder to store the bounding box coordinates of the detected QR code.

    def start_camera(self):
        """Initializes the camera and starts the thread."""
        # Create a VideoCapture object to get video from the webcam.
        self.capture = cv2.VideoCapture(self.device_index)
        # If the camera could not be opened, raise a runtime error.
        if not self.capture.isOpened():
            raise RuntimeError(f"Cannot open webcam (device_index={self.device_index})")
        self.running.set()  # Set the running event, which will allow the `run` method's loop to start.
        self.start()    # Start the thread (this will call the `run` method).

    def run(self):
        """The main loop of the scanner thread."""
        # Loop as long as the running event is set and the QR code has not been detected yet.
        while self.running.is_set() and not self.detected.is_set():
            # Read a single frame from the camera.
            ret, frame = self.capture.read()
            # If the frame was not successfully read, wait a bit and try again.
            if not ret:
                time.sleep(0.05)
                continue
            # Try to decode QR codes from the current frame.
            barcodes = decode(frame)
            # Loop through all the barcodes found in the frame.
            for barcode in barcodes:
                # Check if the data of a found barcode matches our target.
                if barcode.data.decode('utf-8') == CANONICAL_QR_PAYLOAD:
                    # If it matches, get the corner points of the QR code.
                    pts = barcode.polygon
                    # Extract all x and y coordinates from the points.
                    x_coords, y_coords = [p.x for p in pts], [p.y for p in pts]
                    # Calculate the bounding box (min and max coordinates).
                    self.detected_box = (min(x_coords), min(y_coords), max(x_coords), max(y_coords))
                    # Save a copy of the current frame.
                    self.detected_frame = frame.copy()
                    # Set the detected event to signal the main thread.
                    self.detected.set()
                    print("Scanner: canonical QR detected.")
                    # Exit the inner loop since we found what we were looking for.
                    break
            # Display the live camera feed in a window named "Phoenix Aero QR Camera Feed".
            cv2.imshow("Phoenix Aero QR Camera Feed", frame)
            # Check if the 'q' key was pressed to quit the feed.
            if cv2.waitKey(1) & 0xFF == ord('q'):
                # If 'q' is pressed, clear the running event to stop the thread.
                self.running.clear()
                # Exit the loop.
                break
        print("Scanner thread loop finished.")

    def stop(self):
        """Gracefully stops the scanner thread and releases resources."""
        print("Stopping camera and closing window.")
        self.running.clear()    # clears the running event to ensure the `run` loop exits.

        time.sleep(0.2)      # Pause briefly to allow the thread to finish its last cycle.
        # If the camera capture object exists, release the hardware.
        if self.capture: self.capture.release()
        # Safely destroy OpenCV windows, ignoring errors if they don't exist.
        try: cv2.destroyWindow("Phoenix Aero QR Camera Feed")
        except cv2.error: pass
        try: cv2.destroyWindow("Phoenix Aero QR Camera Feed - DETECTED")
        except cv2.error: pass

# ---------- Main mission functions ----------
def arm_and_takeoff(target_alt):
    """Arms the drone and takes off to a specified altitude."""
    print("Waiting for vehicle to become armable...")
    # Wait until the vehicle passes all pre-arm checks.
    while not vehicle.is_armable:
        print("  Waiting for arming pre-checks...")
        time.sleep(1)
    print("Vehicle is armable. Arming motors.")
    vehicle.mode = VehicleMode("GUIDED")    # Set the vehicle mode to GUIDED, which allows control from a script.
    vehicle.armed = True    # Arm the motors.
    # Wait for the arming command to be confirmed.
    while not vehicle.armed:
        print("  Waiting for arming confirmation...")
        time.sleep(1)
    print("Taking off!")
    # Command the vehicle to take off to the target altitude.
    vehicle.simple_takeoff(target_alt)
    # Wait until the vehicle reaches at least 95% of the target altitude.
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f" Altitude: {alt:.1f} m")
        if alt >= target_alt * 0.95: break
        time.sleep(1)
    print("Reached target altitude")

def freeze_and_show_detected(scanner: QRScanner):
    """Displays the static frame where the QR code was detected."""
    # Make a copy of the saved frame to draw on.
    frame = scanner.detected_frame.copy()
    (x1, y1, x2, y2) = scanner.detected_box # gets the coordinates of the bounding box.
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)    # Draw a green rectangle around the detected QR code.
    cv2.putText(frame, "QR DETECTED", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2) # Put text above the rectangle to confirm detection.
    cv2.imshow("Phoenix QR Camera Feed - DETECTED", frame)  # Show the new image in a separate window.
    # Close the old live feed window.
    try: cv2.destroyWindow("Phoenix QR Camera Feed")
    except cv2.error: pass

def align_hover_and_rtl(scanner: QRScanner):
    """
    This is the new final sequence.
    It aligns over the QR, descends to a low hover, waits, ascends, and then RTLs.
    """
    # Check if the camera is available before proceeding.
    if not (scanner.capture and scanner.capture.isOpened()):
        print("ERROR: Camera not available. Commanding RTL without alignment.")
        vehicle.mode = VehicleMode("RTL")
        return

    # --- 1. Alignment Phase ---
    frame, (x1, y1, x2, y2) = scanner.detected_frame, scanner.detected_box  # Get the detected frame and bounding box from the scanner object.
    h, w = frame.shape[:2]  # Get the height and width of the video frame
    center_x, center_y = int((x1 + x2) / 2), int((y1 + y2) / 2)  # Calculate the center of the detected QR code.
    iter_count = 0  # Initialize iteration counter for the alignment loop.
    print("Starting final alignment...")
    # Loop until aligned or the iteration limit is reached.
    while iter_count < ALIGNMENT_ITER_LIMIT:
        img_center_x, img_center_y = w // 2, h // 2     # Calculate the center of the image frame.
        # Calculate the error in pixels between the QR center and the image center.
        dx, dy = center_x - img_center_x, img_center_y - center_y
        # If the error is within the tolerance, alignment is successful.
        if abs(dx) <= ALIGNMENT_PIXEL_TOL and abs(dy) <= ALIGNMENT_PIXEL_TOL:
            print("Alignment within tolerance.")
            break

        px_to_m, max_step = 0.0025, 0.7 # These are tuning values: pixel-to-meter conversion and max speed per step.
        east_offset, north_offset = dx * px_to_m, dy * px_to_m  # Convert pixel error to a movement offset in meters.
        # Limit the speed of the correction step to prevent overshooting.
        step_north = max(-max_step, min(max_step, north_offset))
        step_east = max(-max_step, min(max_step, east_offset))
        send_ned_velocity(step_north, step_east, 0, duration=1.0)   # Send the velocity command to nudge the drone.
        iter_count += 1     # Increment the counter.
    print("Alignment complete.")

    # --- 2. Low Pass / "Touch-and-Go" Phase ---
    current_alt = vehicle.location.global_relative_frame.alt    # Get the drone's current altitude.
    descent_distance = current_alt - CONFIRM_HOVER_ALT  # Calculate how far the drone needs to descend.
    
    # Only descend if the drone is above the target hover altitude.
    if descent_distance > 0:
        descent_speed = 0.5  # Define descent speed in m/s (positive is down).
        descent_duration = descent_distance / descent_speed  # Calculate the duration needed for the descent.
        print(f"Descending to {CONFIRM_HOVER_ALT}m to confirm target.")
        send_ned_velocity(0, 0, descent_speed, duration=descent_duration)  # Send the velocity command to descend.
        time.sleep(descent_duration + 1)    # Wait for the descent to complete, plus a small buffer.

    print(f"Hovering at {CONFIRM_HOVER_ALT}m to confirm location...")
    time.sleep(10)  # Pause (hover) for 10 seconds to simulate confirmation.

    print("Ascending back to safe altitude.")
    ascent_speed = -1.0  # Define ascent speed in m/s (negative is up).
    ascent_duration = descent_distance / abs(ascent_speed)  # Calculate the duration needed for the ascent.
    send_ned_velocity(0, 0, ascent_speed, duration=ascent_duration)  # Send the velocity command to ascend.
    time.sleep(ascent_duration + 2)  # Wait for the ascent to complete, plus a small buffer.

    # --- 3. RTL Phase ---
    print("Target confirmed. Initiating RTL.")
    vehicle.mode = VehicleMode("RTL")   # Set the vehicle mode to Return to Launch.
    print("Mission phase finished. Drone is returning to launch.")

def manual_scan_for_qr(scanner, scan_duration):
    """A fallback function to actively scan for the QR code while hovering."""
    print(f"Starting manual scan for {scan_duration} seconds...")
    start_time = time.time()      # Record the start time.
    # Loop for the specified duration.
    while time.time() - start_time < scan_duration:
        ret, frame = scanner.capture.read()  # Read a frame from the camera.
        if not ret: time.sleep(0.05); continue
        cv2.imshow("Phoenix Aero QR Camera Feed", frame)  # Show the live feed.
        if cv2.waitKey(1) & 0xFF == ord('q'): return False
        # Check the frame for the target QR code.
        for barcode in decode(frame):
            if barcode.data.decode('utf-8') == CANONICAL_QR_PAYLOAD:
                print("QR code found during manual side scan.")
                # If found, save its details to the scanner object.
                pts = barcode.polygon
                scanner.detected_box = (min(p.x for p in pts), min(p.y for p in pts),
                                        max(p.x for p in pts), max(p.y for p in pts))
                scanner.detected_frame = frame.copy()
                # Return True to indicate success.
                return True
    print("Manual scan finished, no QR found in this phase.")
    return False    # returns false if the time runs out and no QR was found.

def goto_and_update_feed(scanner, target_location):
    """Moves the drone to a target location while continuously displaying the camera feed."""
    print(f"Moving to new location while updating feed...")
    simple_goto_location(target_location)   # Command the drone to go to the new location.
    # Loop until the drone arrives.
    while True:
        # If the scanner is active, update the camera feed window.
        if scanner and scanner.capture and scanner.capture.isOpened():
            ret, frame = scanner.capture.read()
            if ret: cv2.imshow("Phoenix Aero QR Camera Feed", frame); cv2.waitKey(1)
        # Check if the drone is within 1.2 meters of the target.
        if distance_between_locations_m(vehicle.location.global_relative_frame, target_location) < 1.2:
            # If it has arrived, break the loop.
            break
        time.sleep(0.1)

# ---------- Execution ----------
# This is the main part of the script that runs the mission logic.
scanner = None  # Initialize the scanner variable to None.
try:
    # 1. Load the target QR code's data from the reference image.
    CANONICAL_QR_PAYLOAD = decode_image_payload(CANONICAL_QR_PATH)
    print("Canonical QR payload loaded:", CANONICAL_QR_PAYLOAD)

    # 2. Arm the drone and take off.
    arm_and_takeoff(TAKEOFF_ALT)
    start_loc = vehicle.location.global_relative_frame      # Record the starting location (home).
    
    # 3. Define the primary flight path (straight line forward).
    forward_target = get_location_meters_metres(start_loc, TARGET_DISTANCE, 0)
    print("Issuing continuous goto to forward target.")
    simple_goto_location(forward_target, groundspeed=3.0)   # Command the drone to start flying along the path.
    detection_happened = False  # A flag to track if detection happened.

    # 4. Main flight loop.
    while True:
        dist = distance_between_locations_m(start_loc, vehicle.location.global_relative_frame)  # Calculate the current distance from the start point.
        print(f"Drone is at {dist:.2f} m from start.")
        # If the drone is far enough along the path and the scanner hasn't started yet...
        if dist >= CAMERA_START_DISTANCE and scanner is None:
            scanner = QRScanner(device_index=0)  # Create a QRScanner object.
            try:
                scanner.start_camera()  # Start the camera and scanner thread.
            except Exception as e:
                print(f"Error starting scanner: {e}"); scanner = None   # If starting the camera fails, print an error and nullify the scanner object.
        # If the scanner object exists and its `detected` event is set...
        if scanner and scanner.detected.is_set():
            detection_happened = True; break    # Set our flag to True and exit the loop.
        # If the drone has reached the end of the search path, exit the loop.
        if dist >= TARGET_DISTANCE - 0.5: break
        time.sleep(0.5)  # Pause before the next progress check.

    # After the loop, stop the scanner thread's `run` loop (but don't release resources yet).
    if scanner: scanner.running.clear(); time.sleep(0.2)

    # 5. Post-flight logic.
    # Case 1: The QR code was successfully detected.
    if detection_happened:
        print("QR detected. Beginning final hover and RTL sequence.")
        freeze_and_show_detected(scanner)   # Show the frame where the QR was found.
        # Call the function to align, hover, and RTL.
        align_hover_and_rtl(scanner) # <-- CALLING THE NEW FUNCTION
    # Case 2: The forward search finished, but no QR was found (and the scanner worked).
    elif scanner:
        print("No detection on forward leg. Starting fallback scan procedure.")
        found_side = False
        back_target = get_location_meters_metres(vehicle.location.global_relative_frame, -2.5, 0)   # Move back 2.5m and scan.
        goto_and_update_feed(scanner, back_target)
        right_target = get_location_meters_metres(vehicle.location.global_relative_frame, 0, 5)     # Move right 5m and scan.
        goto_and_update_feed(scanner, right_target)
        if manual_scan_for_qr(scanner, SIDE_CHECK_TIMEOUT):
            found_side = True
        else:
            # If not found, move left and scan.
            left_target = get_location_meters_metres(vehicle.location.global_relative_frame, 0, -10)    # Move left 10m and scan.
            goto_and_update_feed(scanner, left_target)
            if manual_scan_for_qr(scanner, SIDE_CHECK_TIMEOUT):
                found_side = True
        
        # If the QR was found during the side scan...
        if found_side:
            print("QR found during side scan. Beginning final hover and RTL sequence.")
            freeze_and_show_detected(scanner)
            # Call the function to align, hover, and RTL.
            align_hover_and_rtl(scanner) # <-- CALLING THE NEW FUNCTION
        else:
            # If not found anywhere, give up and go home.
            print("No QR found in the way. Commanding RTL.")
            vehicle.mode = VehicleMode("RTL")
    # Case 3: The scanner failed to start.
    else:
        print("No QR detected and camera not active. Returning to launch.")
        vehicle.mode = VehicleMode("RTL")

    # After the main logic, monitor the vehicle's state until it disarms.
    print("Main mission logic finished. Monitoring vehicle state.")
    if 'vehicle' in locals() and vehicle.armed:
        while vehicle.armed:
             print(f"  Current Mode: {vehicle.mode.name}. Altitude: {vehicle.location.global_relative_frame.alt:.1f}m. Distance from home: {distance_between_locations_m(vehicle.location.global_relative_frame, start_loc):.1f}m")
             time.sleep(2)
    print("Vehicle has disarmed.")

# This `finally` block will execute no matter what happens in the `try` block (success, failure, or error).
finally:
    # Stop the scanner thread and release its resources.
    if scanner: scanner.stop()
    try:
        # A final failsafe: if the script is ending but the drone is still armed...
        if 'vehicle' in locals() and vehicle.is_armable and vehicle.armed:
            print("FAILSAFE: Script ending but vehicle still armed. Commanding RTL.")
            vehicle.mode = VehicleMode("RTL")   # Command it to return to launch.
        # If the vehicle object exists, close the connection.
        if 'vehicle' in locals(): vehicle.close()
    except Exception as e: print(f"Error during final cleanup: {e}")
    # Ensure all OpenCV windows are closed.
    cv2.destroyAllWindows()
    print("Cleanup done. Script finished.")
