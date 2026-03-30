
import cv2
import numpy as np

# 1. Setup
cap = cv2.VideoCapture('/root/subtask1 [optical flow]/data/OPTICAL_FLOW.mp4')
ret, frame1 = cap.read()

if not ret:
    print("Error: Could not read video file.")
    exit()

# Resize for speed
scale = 0.5
frame1 = cv2.resize(frame1, (0,0), fx=scale, fy=scale)

prvs = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)

# Create HSV image for visualization
hsv = np.zeros_like(frame1)
hsv[...,1] = 255



frame_count = 0

while frame_count < (30 * 20):  

    ret, frame2 = cap.read()
    if not ret:
        break

    frame2 = cv2.resize(frame2,(0,0),fx=scale,fy=scale)
    next_gray = cv2.cvtColor(frame2,cv2.COLOR_BGR2GRAY)

    # 2. Calculate Dense Optical Flow (Farneback)
    flow = cv2.calcOpticalFlowFarneback(
        prvs,
        next_gray,
        None,
        0.5,
        3,
        15,
        3,
        5,
        1.2,
        0
    )

    # 3. Convert flow vectors to magnitude and angle
    mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])

    # Map direction to Hue
    hsv[...,0] = ang * 180 / np.pi / 2

    # Map magnitude to Brightness
    hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)

    # Convert HSV → BGR
    rgb_flow = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)

    # Overlay motion heatmap with original frame
    combined = cv2.addWeighted(frame2,0.7,rgb_flow,0.3,0)

    # Display result
    cv2.imshow("Dense Optical Flow", combined)

    # Update frame
    prvs = next_gray
    frame_count += 1

    # Press q to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
