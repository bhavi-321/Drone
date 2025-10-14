# Python Codes

- `drone_cam_me.py` : drone takes off to a specified altitude and tries to centre at a pre-initialised qr code. RTLs if qr code falls under central tolerance. code uses PID to continuously give roll and pitch velocities to the drone
- `final.py` : drone takes off to a specified altitude, goes to a specified location, opens camera and searches for qr code around, centres at the qr code with constant velocity commands provided using PID controller class. after centering, camera feed closes, drone executes landing followed by RTL
- `follow_red_xy.py` : drone takes off to 10m and tries to centre at the largest red object detected in the frame using yaw movements and changing altitude
- `red_drone.py` : drone flies to a point and executes RTL if it detects red through camera during its flight, else it executes RTL after reaching the point
