from ultralytics import YOLO
from deep_sort_realtime.deepsort_tracker import DeepSort
import cv2
import numpy as np
import sys
import os

# Initialize YOLO model (only person class)
model = YOLO("yolov8x.pt")

# Initialize DeepSORT tracker
tracker = DeepSort(
    max_age=60,
    n_init=3,
    max_iou_distance=0.8,
    nn_budget=120,
    embedder='mobilenet'
)


# Drone Video Feed - choose source
# Priority: command-line arg (sys.argv[1]) -> 'video.mp4' if present -> fallback to camera 0
if len(sys.argv) > 1:
    source = sys.argv[1]
else:
    if os.path.exists('video.mp4'):
        source = 'video.mp4'
    else:
        print("Warning: 'video.mp4' not found in the working directory.")
        print("Falling back to camera 0. To use a file, run: python human_tracking.py path\\to\\your_video.mp4")
        source = 0

cap = cv2.VideoCapture(source)

if not cap.isOpened():
    print(f"ERROR: Unable to open video source: {source}")
    sys.exit(1)

# No output writer by default
writer = None

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    height, width = frame.shape[:2]
    detections = []

    # Run YOLO only for "person" class
    results = model(frame, verbose=False)[0]

    for box in results.boxes:
        cls = int(box.cls[0])
        if model.names[cls] == "person":  # only detect humans
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            detections.append(([x1, y1, x2 - x1, y2 - y1], conf, cls))

    
    # Tracking with DeepSort
    tracks = tracker.update_tracks(detections, frame=frame)

    for track in tracks:
        if not track.is_confirmed():
            continue
        track_id = track.track_id
        ltrb = track.to_ltrb()
        x1, y1, x2, y2 = map(int, ltrb)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, f'ID {track_id}', (x1, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

    cv2.imshow("Human Detection & Tracking", frame)

    # Playback delay based on FPS for proper video speed (fallback to 1 ms)
    current_fps = cap.get(cv2.CAP_PROP_FPS)
    delay = int(1000 / current_fps) if current_fps and current_fps > 0 else 1
    if cv2.waitKey(delay) & 0xFF == ord('q'):
        break

    # write frame to output file if requested
    if writer is not None:
        writer.write(frame)

cap.release()
if writer is not None:
    writer.release()
cv2.destroyAllWindows()