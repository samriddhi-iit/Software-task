# Manual Lucas-Kanade Optical Flow (From Scratch)

- Objective
    
    Implement the Lucas-Kanade Optical Flow algorithm from scratch (without built-in OpenCV functions) to compute motion vectors and visualize them using arrows on video frames.
    
- Input
    - Video file (`.mp4`)
    - Continuous frames from video stream
- Output
    - Sparse motion vectors (arrows)
    - Motion visualization on selected grid points
- The overall procedure is explained with the help of a flowchart
    
    ```mermaid
    flowchart TD
    	A["Video"] --> B["Read frame"]
    	
    	B --> C["Convert to grayscale"]
    	C --> D["Compute gradients (Ix, Iy, It)"]
    	D --> E["Create a local window"]
    	E --> F["Solve least squares for (u, v)"]
    	F --> G["Draw motion arrows"]
    	G --> H["Display frame"]
    	
    ```
    
    ### ▶ Step 1: Video Setup
    
    ```
    cap=cv2.VideoCapture('path_to_video')
    fps=cap.get(cv2.CAP_PROP_FPS)
    max_frames=int(10*fps)
    ```
    
    - Loads video
    - Limits processing to 10 seconds
    
    ---
    
    ### ▶ Step 2: First Frame Preprocessing
    
    ```
    frame1=cv2.resize(frame1, (0,0),fx=scale,fy=scale)
    prev_gray=cv2.cvtColor(frame1,cv2.COLOR_BGR2GRAY)
    ```
    
    - Resize for speed
    - Convert to grayscale
    
    ---
    
    ### ▶ Step 3: Grid Point Initialization
    
    ```
    step=16
    y,x=np.mgrid[...]
    points=np.stack((x,y),axis=-1)
    ```
    
    - Creates evenly spaced tracking points
    - Reduces computation vs dense flow
    
    ---
    
    ### ▶ Step 4: Compute Gradients
    
    ```
    Ix=cv2.Sobel(I1,cv2.CV_64F,1,0)
    Iy=cv2.Sobel(I1,cv2.CV_64F,0,1)
    It=I2-I1
    ```
    
    - `Ix` → horizontal gradient
    - `Iy` → vertical gradient
    - `It` → temporal change
    
    ---
    
    ### ▶ Step 5: Build Linear System
    
    Lucas-Kanade equation:
    
    ```
    Ix * u + Iy * v + It = 0
    ```
    
    Converted to:
    
    ```
    A [u v]^T = b
    ```
    
    Where:
    
    - `A = [Ix Iy]`
    - `b = -It`
    
    ---
    
    ### ▶ Step 6: Solve Using Least Squares
    
    ```
    nu=np.linalg.pinv(A.T@A)@ (A.T@b)
    ```
    
    - Computes motion `(u, v)`
    - Uses pseudo-inverse for stability
    
    ---
    
    ### ▶ Step 7: Handle Edge Cases
    
    ```
    ifout_of_bounds:
    flows.append([0,0])
    ```
    
    - Avoids invalid regions near borders
    
    ---
    
    ### ▶ Step 8: Visualization
    
    ```
    x2,y2= (p+v*10.0)
    ```
    
    - Scales motion vectors for visibility
    
    ---
    
    ### ▶ Step 9: Filter Noise
    
    ```
    if0.05<mag<50:
    ```
    
    - Removes:
        - static noise
        - extreme outliers
    
    ---
    
    ### ▶ Step 10: Draw Arrows
    
    ```
    cv2.arrowedLine(...)
    cv2.circle(...)
    ```
    
    - Arrows → direction & magnitude
    - Circles → tracked points
    
    ---
    
    ### ▶ Step 11: Update Frame
    
    ```
    prev_gray=next_gray
    ```
    
    - Moves forward in time
    
    ---
    
    ### ▶ Step 12: Exit & Cleanup
    
    ```
    cap.release()
    cv2.destroyAllWindows()
    ```
