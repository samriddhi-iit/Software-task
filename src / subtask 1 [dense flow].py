import cv2
import numpy as np

# 1. Setup
cap = cv2.VideoCapture('/root/subtask1 [optical flow]/data/OPTICAL_FLOW.mp4')
fps = cap.get(cv2.CAP_PROP_FPS)
max_frames = int(10 * fps) # Limit to 10 seconds

ret, frame1 = cap.read()
if not ret: 
    print("Error: Could not read video.")
    exit()

scale = 0.5
frame1 = cv2.resize(frame1, (0,0), fx=scale, fy=scale)
prev_gray = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)

# Grid of points for line visualization (Sampling every 16 pixels)
step = 16
y, x = np.mgrid[step/2:frame1.shape[0]:step, step/2:frame1.shape[1]:step].reshape(2, -1).astype(int)
points = np.stack((x, y), axis=-1).astype(np.float32)

def manual_lk(I1, I2, pts, win=7):
    """From-scratch Lucas-Kanade: Solving for [u, v] using spatial/temporal gradients."""
    # Compute gradients
    Ix = cv2.Sobel(I1, cv2.CV_64F, 1, 0, ksize=3)
    Iy = cv2.Sobel(I1, cv2.CV_64F, 0, 1, ksize=3)
    It = I2.astype(np.float64) - I1.astype(np.float64)
    
    h_win = win // 2
    flows = []
    
    for pt in pts:
        px, py = int(pt[0]), int(pt[1])
        if py-h_win < 0 or py+h_win >= I1.shape[0] or px-h_win < 0 or px+h_win >= I1.shape[1]:
            flows.append([0, 0])
            continue
            
        # Extract local windows
        ix = Ix[py-h_win:py+h_win, px-h_win:px+h_win].flatten()
        iy = Iy[py-h_win:py+h_win, px-h_win:px+h_win].flatten()
        it = It[py-h_win:py+h_win, px-h_win:px+h_win].flatten()
        
        # Build System: A * [u, v]^T = b
        A = np.vstack((ix, iy)).T
        b = -it.reshape(-1, 1)
        
        # Solve via Least Squares
        try:
            # ATAx = ATb
            nu = np.linalg.pinv(A.T @ A) @ (A.T @ b)
            flows.append(nu.flatten())
        except:
            flows.append([0, 0])
            
    return np.array(flows)



frame_count = 0
print(f"Running simulation for {max_frames} frames (10s)... Press 'q' to stop.")

while frame_count < max_frames:
    ret, frame2 = cap.read()
    if not ret: break
    
    frame2 = cv2.resize(frame2, (0,0), fx=scale, fy=scale)
    next_gray = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    # Calculate vectors
    vectors = manual_lk(prev_gray, next_gray, points)

    # Visualization
    vis = frame2.copy()
    for p, v in zip(points, vectors):
        x1, y1 = p.astype(int)
        
        #  MULTIPLIER: v * 10.0 makes tiny distant movements visible
        x2, y2 = (p + v * 10.0).astype(int)
        
        # LOWER THRESHOLD: Reduced from 0.2 to 0.05 to catch distant cars
        mag = np.linalg.norm(v)
        if 0.05 < mag < 50: # Ignore pure static and extreme outliers
            cv2.arrowedLine(vis, (x1, y1), (x2, y2), (0, 255, 0), 1, tipLength=0.3)
            cv2.circle(vis, (x1, y1), 1, (0, 0, 255), -1)

    cv2.imshow('10s Simulation - Manual LK (Arrows)', vis)
    
    prev_gray = next_gray
    frame_count += 1
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
    

print("Simulation complete.")
