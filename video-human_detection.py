import cv2
import numpy as np
import onnxruntime as ort
from deep_sort_realtime.deepsort_tracker import DeepSort
import os

# --- Config ---
ONNX_MODEL = "yolov8n.onnx"   # path to your ONNX model
INPUT_VIDEO = "dronev2.mp4"      # INPUT VIDEO PATH
OUTPUT_VIDEO = "output_tracked.mp4"  # OUTPUT VIDEO PATH
IMG_SIZE = 640                 # inference size used at export
CONF_THR = 0.20                # Lower threshold for small drone-view humans
IOU_THR = 0.45
FRAME_SKIP = 1                 # Process every frame for better tracking (set to 2-3 if too slow)
MAX_COSINE_DISTANCE = 0.3      # Slightly relaxed for drone view variation

# --- Helpers ---
def letterbox(img, new_shape=(IMG_SIZE, IMG_SIZE), color=(114,114,114)):
    """Resize and pad image to square (like YOLO letterbox)."""
    h0, w0 = img.shape[:2]
    r = min(new_shape[0]/h0, new_shape[1]/w0)
    new_unpad = (int(round(w0 * r)), int(round(h0 * r)))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    dw, dh = dw // 2, dh // 2
    img_resized = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = dh, new_shape[0] - new_unpad[1] - dh
    left, right = dw, new_shape[1] - new_unpad[0] - dw
    img_padded = cv2.copyMakeBorder(img_resized, top, bottom, left, right,
                                    cv2.BORDER_CONSTANT, value=color)
    return img_padded, r, (left, top)

def xywh_to_xyxy(cx, cy, w, h):
    x1 = cx - w/2; y1 = cy - h/2
    x2 = cx + w/2; y2 = cy + h/2
    return [x1, y1, x2, y2]

# --- Validate input video ---
if not os.path.exists(INPUT_VIDEO):
    raise FileNotFoundError(f"Input video not found: {INPUT_VIDEO}")

# --- Load ONNX model ---
print(f"Loading ONNX model: {ONNX_MODEL}")
session = ort.InferenceSession(ONNX_MODEL, providers=['CPUExecutionProvider'])
input_name = session.get_inputs()[0].name
in_shape = session.get_inputs()[0].shape
print(f"ONNX input shape: {in_shape} -> {input_name}")

# Model image size from ONNX (if available), fallback to IMG_SIZE
try:
    model_img_h = int(in_shape[2]) if in_shape[2] is not None else IMG_SIZE
    model_img_w = int(in_shape[3]) if in_shape[3] is not None else IMG_SIZE
except Exception:
    model_img_h = model_img_w = IMG_SIZE

print(f"Model inference size: {model_img_w}x{model_img_h}")

# --- Initialize Tracker (tuned for drone view) ---
tracker = DeepSort(
    max_age=30,           # Keep tracks for 30 frames without detection
    n_init=3,             # Confirm track after 3 consecutive detections
    max_cosine_distance=MAX_COSINE_DISTANCE,
    nn_budget=100
)

# --- Open input video ---
print(f"Opening input video: {INPUT_VIDEO}")
cap = cv2.VideoCapture(INPUT_VIDEO)
if not cap.isOpened():
    raise RuntimeError(f"Cannot open video source: {INPUT_VIDEO}")

# Get video properties
fps = cap.get(cv2.CAP_PROP_FPS)
total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

print(f"Video properties: {w}x{h} @ {fps:.2f} FPS, Total frames: {total_frames}")

# --- Create output video writer ---
fourcc = cv2.VideoWriter_fourcc(*"mp4v")
out_writer = cv2.VideoWriter(OUTPUT_VIDEO, fourcc, fps, (w, h))
if not out_writer.isOpened():
    raise RuntimeError(f"Cannot create output video: {OUTPUT_VIDEO}")

print(f"Output video: {OUTPUT_VIDEO}")
print("Processing...")

frame_id = 0
processed_frames = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    frame_id += 1
    
    # Progress indicator
    if frame_id % 30 == 0:
        progress = (frame_id / total_frames) * 100 if total_frames > 0 else 0
        print(f"Progress: {frame_id}/{total_frames} frames ({progress:.1f}%)")
    
    # Skip frames if configured
    if frame_id % FRAME_SKIP != 0:
        out_writer.write(frame)
        continue
    
    processed_frames += 1

    # Preprocess frame
    img, r, (pad_x, pad_y) = letterbox(frame, new_shape=(model_img_w, model_img_h))
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img_norm = img_rgb.astype(np.float32) / 255.0
    inp = np.transpose(img_norm, (2, 0, 1))[np.newaxis, :]

    # ONNX inference
    outputs = session.run(None, {input_name: inp})
    out0 = np.array(outputs[0])

    # --- Decode output layout ---
    if out0.ndim == 3 and out0.shape[1] == 84:
        pred = out0[0].T  # -> (num_boxes, 84)
    else:
        pred = out0.squeeze(0) if out0.ndim == 3 else out0

    boxes = []
    confidences = []
    class_ids = []

    for row in pred:
        if row.size < 5:
            continue
        cx, cy, wbox, hbox = float(row[0]), float(row[1]), float(row[2]), float(row[3])
        class_scores = row[4:]
        if class_scores.size == 0:
            continue
        cls = int(np.argmax(class_scores))
        score = float(class_scores[cls])

        if score < CONF_THR:
            continue

        # Scale normalized coords to model image size
        if max(cx, cy, wbox, hbox) <= 1.0 + 1e-6:
            cx *= model_img_w
            cy *= model_img_h
            wbox *= model_img_w
            hbox *= model_img_h

        x1, y1, x2, y2 = xywh_to_xyxy(cx, cy, wbox, hbox)

        # Undo letterbox padding & scale back to original frame coords
        x1 = (x1 - pad_x) / r
        y1 = (y1 - pad_y) / r
        x2 = (x2 - pad_x) / r
        y2 = (y2 - pad_y) / r

        # Clamp to frame boundaries
        x1 = max(0.0, min(x1, w - 1))
        y1 = max(0.0, min(y1, h - 1))
        x2 = max(0.0, min(x2, w - 1))
        y2 = max(0.0, min(y2, h - 1))

        boxes.append([x1, y1, x2, y2])
        confidences.append(float(score))
        class_ids.append(int(cls))

    # --- NMS ---
    boxes_xywh = [[int(b[0]), int(b[1]), int(b[2] - b[0]), int(b[3] - b[1])] 
                  for b in boxes] if boxes else []

    keep_idxs = []
    if len(boxes_xywh) > 0:
        idxs = cv2.dnn.NMSBoxes(boxes_xywh, confidences, CONF_THR, IOU_THR)
        if idxs is not None:
            idxs = np.array(idxs)
            keep_idxs = idxs.flatten().tolist() if idxs.ndim == 2 else idxs.tolist()

    # --- Filter persons (COCO class id 0) ---
    person_dets = []
    for i in keep_idxs:
        if class_ids[i] != 0:  # Only track persons
            continue
        x1, y1, x2, y2 = boxes[i]
        w_det = x2 - x1
        h_det = y2 - y1
        if w_det <= 2 or h_det <= 2:  # Filter tiny boxes
            continue
        person_dets.append(([int(x1), int(y1), int(w_det), int(h_det)], 
                           confidences[i], 'person'))

    # --- Update tracker ---
    tracks = tracker.update_tracks(person_dets, frame=frame)

    # --- Draw tracks on frame ---
    for t in tracks:
        if not t.is_confirmed():
            continue
        
        tid = t.track_id
        left, top, right, bottom = map(int, t.to_ltrb())
        
        # Draw bounding box (green for confirmed tracks)
        cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
        
        # Draw ID label with background
        label = f"ID:{tid}"
        label_size, _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        label_w, label_h = label_size
        
        # Background rectangle for text
        cv2.rectangle(frame, (left, top - label_h - 10), 
                     (left + label_w + 5, top), (0, 255, 0), -1)
        
        # White text
        cv2.putText(frame, label, (left + 2, top - 5), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    # Write frame to output video
    out_writer.write(frame)

# --- Cleanup ---
cap.release()
out_writer.release()

print(f"\n✓ Processing complete!")
print(f"  Total frames: {frame_id}")
print(f"  Processed frames: {processed_frames}")
print(f"  Output saved to: {OUTPUT_VIDEO}")