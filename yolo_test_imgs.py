from ultralytics import YOLO
import cv2
import numpy as np
import argparse
import glob
import os
import sys

# Simple image tester for YOLOv8 (person class)
model = YOLO("yolov8x.pt")


def iter_image_paths(src):
    if os.path.isfile(src):
        yield src
        return
    if os.path.isdir(src):
        patterns = ["*.jpg", "*.jpeg", "*.png", "*.bmp", "*.tif", "*.tiff"]
        for p in patterns:
            for f in sorted(glob.glob(os.path.join(src, p))):
                yield f
        return
    for f in sorted(glob.glob(src)):
        yield f


def main():
    parser = argparse.ArgumentParser(description="Simple YOLO image tester")
    parser.add_argument("--src", "-s", default="images", help="Image folder, glob (e.g. 'images/*.jpg') or single file")
    parser.add_argument("--out", "-o", default="out", help="Output folder for annotated images")
    parser.add_argument("--show", action="store_true", help="Show images while processing")
    parser.add_argument("--conf", type=float, default=0.5, help="Confidence threshold")
    parser.add_argument("--imgsz", type=int, default=640, help="YOLO input size")
    parser.add_argument("--sharpen", action="store_true", help="Apply unsharp-mask sharpening before inference")
    parser.add_argument("--sharpen-amt", type=float, default=1.0, help="Sharpening amount (default 1.0)")
    parser.add_argument("--sharpen-sigma", type=float, default=1.0, help="Gaussian sigma for unsharp mask (default 1.0)")
    parser.add_argument("--upscale", type=float, default=1.0, help="Upscale factor before inference (e.g. 2.0)")
    parser.add_argument("--clahe", action="store_true", help="Apply CLAHE (adaptive histogram equalization) on luminance before inference")
    args = parser.parse_args()

    # Running on CPU (no CUDA) — ensure model stays on CPU

    os.makedirs(args.out, exist_ok=True)
    image_paths = list(iter_image_paths(args.src))
    if not image_paths:
        print(f"No images found in: {args.src}")
        sys.exit(1)

    for p in image_paths:
        img = cv2.imread(p)
        if img is None:
            print(f"Failed to read {p}")
            continue

        proc = img.copy()
        h0, w0 = img.shape[:2]
        scale = float(args.upscale) if args.upscale and args.upscale > 1.0 else 1.0
        if scale != 1.0:
            new_w, new_h = int(w0 * scale), int(h0 * scale)
            proc = cv2.resize(proc, (new_w, new_h), interpolation=cv2.INTER_LINEAR)

        # CLAHE on luminance (optional)
        if args.clahe:
            ycrcb = cv2.cvtColor(proc, cv2.COLOR_BGR2YCrCb)
            y, cr, cb = cv2.split(ycrcb)
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            y = clahe.apply(y)
            ycrcb = cv2.merge((y, cr, cb))
            proc = cv2.cvtColor(ycrcb, cv2.COLOR_YCrCb2BGR)

        # Unsharp mask sharpening (optional)
        if args.sharpen:
            blurred = cv2.GaussianBlur(proc, (0, 0), args.sharpen_sigma)
            proc = cv2.addWeighted(proc, 1.0 + args.sharpen_amt, blurred, -args.sharpen_amt, 0)

        results = model(proc, verbose=False, conf=args.conf, imgsz=args.imgsz)[0]

        for box in results.boxes:
            cls = int(box.cls[0])
            if model.names[cls] != 'person':
                continue
            x1f, y1f, x2f, y2f = map(float, box.xyxy[0])
            conf = float(box.conf[0])
            # map coordinates back to original image size if we upscaled
            if scale != 1.0:
                x1f /= scale
                y1f /= scale
                x2f /= scale
                y2f /= scale
            x1, y1, x2, y2 = map(int, (x1f, y1f, x2f, y2f))
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(img, f"person {conf:.2f}", (x1, y1 - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1)

        out_path = os.path.join(args.out, os.path.basename(p))
        cv2.imwrite(out_path, img)

        if args.show:
            cv2.imshow('YOLO Image Test', img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()