# import cv2 as cv
# import numpy as np

# cap = cv.VideoCapture(0)
# while True:
#     isTrue, frame = cap.read()
#     cv.imshow('video', frame)
#     b,g,r = cv.split(frame)
#     b_inverted = cv.bitwise_not(b)
#     g_inverted = cv.bitwise_not(g)
#     frame_processed = cv.merge((b_inverted, g_inverted, r))
#     cv.imshow("Original Frame", frame)
#     cv.imshow("Processed Frame", frame_processed)
#     if cv.waitKey(1) & 0xFF == ord('d'):
#         break
# cap.release()
# cv.destroyAllWindows()


import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    h, w, _ = frame.shape

    #  region of interest (ROI) 
    roi_x_start, roi_y_start = 0, 0
    roi_x_end, roi_y_end = w // 2, h // 2   # 25% region
    
    roi = frame[roi_y_start:roi_y_end, roi_x_start:roi_x_end]

    # Convert BGR to HSV
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Define red color range in HSV
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    # Create mask for red
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = mask1 | mask2

    # Find contours of detected red regions
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > 500:  # filter small noise
            x, y, w_box, h_box = cv2.boundingRect(cnt)
            # Draw bounding box (adjusting to full frame coordinates)
            cv2.rectangle(frame, (x + roi_x_start, y + roi_y_start), (x + roi_x_start + w_box, y + roi_y_start + h_box), (0, 255, 0), 2)

    # Draw boundary around ROI itself
    cv2.rectangle(frame, (roi_x_start, roi_y_start), (roi_x_end, roi_y_end), (255, 0, 0), 2)

    # Show the output
    cv2.imshow("Red Detection in ROI", frame)

    if cv2.waitKey(1) & 0xFF == 27:  # ESC to exit
        break

cap.release()
cv2.destroyAllWindows()
