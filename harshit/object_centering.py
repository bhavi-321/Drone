import cv2
import numpy as np


Kp_xy = 0.6
Kp_z = 1.2
Vmax = 0.5
eps_xy = 0.02
eps_z = 0.1
target_area = None  # Reference area when QR is first detected

cap = cv2.VideoCapture(0)
detector = cv2.QRCodeDetector()

def draw_dynamic_grid(frame, scale_factor):
    h, w, _ = frame.shape
    # Scale factor modifies step size (bigger QR = larger cells)
    base_step = 40
    step = int(np.clip(base_step * scale_factor, 10, 100))
    for y in range(0, h, step):
        cv2.line(frame, (0, y), (w, y), (60, 60, 60), 1)
    for x in range(0, w, step):
        cv2.line(frame, (x, 0), (x, h), (60, 60, 60), 1)
    return frame

while True:
    ret, frame = cap.read()
    if not ret:
        break

    h, w, _ = frame.shape
    cx, cy = w // 2, h // 2

    # Detect QR code
    data, points, _ = detector.detectAndDecode(frame)

    if points is not None:
        pts = points[0].astype(int)
        x_min, y_min = np.min(pts, axis=0)
        x_max, y_max = np.max(pts, axis=0)
        qr_cx, qr_cy = int(np.mean(pts[:, 0])), int(np.mean(pts[:, 1]))

        qr_area = (x_max - x_min) * (y_max - y_min)
        if target_area is None:
            target_area = qr_area  # baseline area

        # Depth scaling: how big QR looks relative to start
        scale_factor = target_area / qr_area
        scale_factor = np.clip(scale_factor, 0.5, 2.0)  # limit zoom effect

        # Draw dynamic grid
        frame = draw_dynamic_grid(frame, scale_factor)

        # Normalized offset
        nx = (qr_cx - cx) / cx
        ny = (qr_cy - cy) / cy

        # Depth velocity (vz)
        depth_ratio = qr_area / target_area
        vx = -Kp_xy * ny * Vmax
        vy =  Kp_xy * nx * Vmax
        vz = -Kp_z * (depth_ratio - 1.0) * Vmax

        # Centered condition
        if abs(nx) < eps_xy and abs(ny) < eps_xy and abs(depth_ratio - 1) < eps_z:
            color = (0, 255, 0)
            vx = vy = vz = 0
            status = "CENTERED"
        else:
            color = (0, 0, 255)
            status = "TRACKING"

        # Draw bounding box + center points
        cv2.polylines(frame, [pts], True, color, 2)
        cv2.circle(frame, (qr_cx, qr_cy), 6, color, -1)
        cv2.circle(frame, (cx, cy), 6, (255, 255, 255), -1)

        # Display info
        cv2.putText(frame, f"Status: {status}", (30, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.putText(frame, f"vx: {vx:.3f} m/s", (30, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"vy: {vy:.3f} m/s", (30, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"vz: {vz:.3f} m/s", (30, 120),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"Grid scale: {scale_factor:.2f}", (30, 150),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (180, 180, 180), 2)

    else:
        frame = draw_dynamic_grid(frame, 1.0)
        cv2.putText(frame, "No QR Detected", (30, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    # Crosshair
    cv2.line(frame, (cx, 0), (cx, h), (255, 255, 255), 1)
    cv2.line(frame, (0, cy), (w, cy), (255, 255, 255), 1)

    cv2.imshow("QR Tracker - Dynamic Grid", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
