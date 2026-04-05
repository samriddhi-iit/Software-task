import cv2
import numpy as np
import tempfile
import pathlib
import pybullet as p
import pybullet_data
import time

def get_car_camera(car_id):
    """
    Captures an RGB frame from the car's perspective.
    Returns: A numpy array (uint8) in RGB format.
    """
    # 1. Get the car's current position and orientation
    pos, orn = p.getBasePositionAndOrientation(car_id)
    rot_matrix = p.getMatrixFromQuaternion(orn)
    
    # 2. Define the 'Forward' vector based on car rotation
    # In PyBullet's racecar, the forward axis is usually X
    forward_vec = [rot_matrix[0], rot_matrix[3], rot_matrix[6]]
    
    # 3. Position the camera: 0.5 units forward, 0.3 units up from the car center
    camera_pos = [
        pos[0] + 0.5 * forward_vec[0], 
        pos[1] + 0.5 * forward_vec[1], 
        pos[2] + 0.3
    ]
    
    # 4. Define where the camera is looking (1 meter ahead)
    target_pos = [
        camera_pos[0] + forward_vec[0], 
        camera_pos[1] + forward_vec[1], 
        camera_pos[2]
    ]
    
    # 5. Compute Matrices
    view_matrix = p.computeViewMatrix(camera_pos, target_pos, [0, 0, 1])
    # FOV=90 is standard for navigation; Aspect=1.33 for a 320x240 view
    proj_matrix = p.computeProjectionMatrixFOV(90, 1.33, 0.01, 100)
    
    # 6. Get the image data from PyBullet
    # Using ER_TINY_RENDERER for speed (standard for CPU)
    _, _, rgb, _, _ = p.getCameraImage(
        width=320, 
        height=240, 
        viewMatrix=view_matrix, 
        projectionMatrix=proj_matrix,
        renderer=p.ER_TINY_RENDERER
    )
    
    # 7. Format the image for OpenCV (RGBA -> RGB)
    frame = np.reshape(rgb, (240, 320, 4))[:, :, :3]
    return frame.astype(np.uint8)

def compute_lk_flow(prev_img, curr_img, points, window_size=21):
    """
    Manual implementation of the Lucas-Kanade tracker.
    """
    # 1. Compute Gradients
    # Ix: Horizontal change, Iy: Vertical change, It: Change over time
    Ix = cv2.Sobel(prev_img, cv2.CV_64F, 1, 0, ksize=3)
    Iy = cv2.Sobel(prev_img, cv2.CV_64F, 0, 1, ksize=3)
    It = curr_img.astype(np.float64) - prev_img.astype(np.float64)

    w = window_size // 2
    new_points = []
    vectors = []
    status = []

    for pt in points:
        x, y = pt.ravel()
        x, y = int(x), int(y)

        # Ensure window stays within image boundaries
        if y-w < 0 or y+w+1 >= prev_img.shape[0] or x-w < 0 or x+w+1 >= prev_img.shape[1]:
            status.append(0)
            continue

        # 2. Extract local windows (The Neighborhood)
        ix = Ix[y-w:y+w+1, x-w:x+w+1].flatten()
        iy = Iy[y-w:y+w+1, x-w:x+w+1].flatten()
        it = It[y-w:y+w+1, x-w:x+w+1].flatten()

        # 3. Build the A matrix and b vector
        # A = [[sum(Ix^2), sum(Ix*Iy)], [sum(Ix*Iy), sum(Iy^2)]]
        A = np.array([
            [np.sum(ix**2), np.sum(ix*iy)],
            [np.sum(ix*iy), np.sum(iy**2)]
        ])
        
        # b = [[-sum(Ix*It)], [-sum(Iy*It)]]
        b = np.array([[-np.sum(ix*it)], [-np.sum(iy*it)]])

        # 4. Solve for displacement (u, v) using Least Squares
        try:
            # pinv (pseudo-inverse) handles cases where the matrix is singular (no texture)
            nuv = np.linalg.pinv(A) @ b
            u, v = nuv.ravel()
            
            new_points.append([x + u, y + v])
            vectors.append([u, v])
            status.append(1)
        except np.linalg.LinAlgError:
            status.append(0)

    return np.array(vectors), np.array(new_points).reshape(-1, 1, 2), np.array(status)

def detect_obstacle(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    lower_yellow = np.array([20,100,100])
    upper_yellow = np.array([35,255,255])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    left  = np.sum(mask[:, :160])
    right = np.sum(mask[:, 160:])
    return left, right

def make_obstacle_texture(size: int = 128, tile: int = 10) -> str:
    img    = np.zeros((size, size, 3), dtype=np.uint8)
    black  = np.array([0,   0,   0],   dtype=np.uint8)
    yellow = np.array([0,   215, 255], dtype=np.uint8) 

    for row in range(size):
        for col in range(size):
            img[row, col] = black if (row // tile + col // tile) % 2 == 0 else yellow

    tex_path = pathlib.Path(tempfile.gettempdir()) / "obs_texture.png"
    ok = cv2.imwrite(str(tex_path), img)
    if not ok:
        raise RuntimeError(f"cv2.imwrite failed writing texture to: {tex_path}")

    print(f"[Texture] Saved yellow/black checkerboard → {tex_path}")
    return str(tex_path)

def create_road_and_obstacles():
    tex_path = make_obstacle_texture(size=128, tile=10)
    tex_id   = p.loadTexture(tex_path)

    # Road surface
    rv = p.createVisualShape(p.GEOM_BOX, halfExtents=[16.66, 1.16, 0.01], rgbaColor=[0.15, 0.15, 0.15, 1])
    rc = p.createCollisionShape(p.GEOM_BOX, halfExtents=[16.66, 1.16, 0.01])
    p.createMultiBody(0, rc, rv, [16.66, 0, 0.01])

    # Lane markings
    for x in np.arange(0, 34, 0.4):
        lv = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.12, 0.03, 0.01], rgbaColor=[1, 1, 1, 1])
        p.createMultiBody(0, -1, lv, [x,  0.0,  0.02])
        p.createMultiBody(0, -1, lv, [x,  0.85, 0.02])
        p.createMultiBody(0, -1, lv, [x, -0.85, 0.02])

    # Slalom obstacles
    obs_extents = [0.25, 0.45, 0.35]
    for i, x in enumerate(range(6, 30, 6)):
        y      = 0.38 if i % 2 == 0 else -0.38
        ov     = p.createVisualShape(p.GEOM_BOX, halfExtents=obs_extents, rgbaColor=[1, 1, 1, 1])
        oc     = p.createCollisionShape(p.GEOM_BOX, halfExtents=obs_extents)
        obs_id = p.createMultiBody(10, oc, ov, [x, y, 0.35])
        p.changeVisualShape(obs_id, -1, textureUniqueId=tex_id)

    # End wall
    wv = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5, 1.16, 0.5], rgbaColor=[0.1, 0.3, 0.9, 1])
    wc = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 1.16, 0.5])
    p.createMultiBody(0, wc, wv, [31.66, 0, 0.5])

def create_car(start_pos=None, start_orn=None, global_scaling=1.8):
    if start_pos is None: start_pos = [0, 0, 0.25]
    if start_orn is None: start_orn = p.getQuaternionFromEuler([0, 0, 0])
    car_id = p.loadURDF("racecar/racecar.urdf", start_pos, start_orn, globalScaling=global_scaling)
    p.changeDynamics(car_id, -1, ccdSweptSphereRadius=0.1)
    steering_joints, motor_joints = [], []
    for i in range(p.getNumJoints(car_id)):
        name = p.getJointInfo(car_id, i)[1].decode('utf-8')
        if 'steer' in name.lower(): steering_joints.append(i)
        elif 'wheel' in name.lower(): motor_joints.append(i)
    return car_id, steering_joints, motor_joints

def setup_simulation(dt=1.0 / 60.0, settle_frames=60, gui=True):
    mode = p.GUI if gui else p.DIRECT
    p.connect(mode)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(dt)
    p.loadURDF("plane.urdf")
    create_road_and_obstacles()
    car_id, steering_joints, motor_joints = create_car()
    for _ in range(settle_frames):
        p.stepSimulation()
        time.sleep(dt)
    return car_id, steering_joints, motor_joints

feature_params = dict(maxCorners=150, qualityLevel=0.01, minDistance=10, blockSize=7)

if __name__ == "__main__":
    car_id, steer_j, motor_j = setup_simulation()
    prev_gray = None
    prev_pts = None

    try:
        while True:
            frame = get_car_camera(car_id)
            curr_gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
            debug_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            
            for motor in motor_j:
                p.setJointMotorControl2(car_id, motor, p.VELOCITY_CONTROL, targetVelocity=6, force=10)

            steer_angle = 0.0

            if prev_gray is not None and prev_pts is not None:
                vectors, next_pts, status = compute_lk_flow(prev_gray, curr_gray, prev_pts)

                if status is not None and next_pts is not None and len(status) == len(next_pts):
                    status = status.reshape(-1)
                    good_new = next_pts[status == 1]
                    good_vec = vectors[status == 1]

                    if len(good_new) > 5:
                        mid_x = 160
                        pts_x = good_new.reshape(-1, 2)[:, 0]
                        dx = good_vec[:, 0]
                        
                        flow_left = np.sum(np.abs(dx[pts_x < mid_x]))
                        flow_right = np.sum(np.abs(dx[pts_x >= mid_x]))
                        
                        steer_angle = (flow_left - flow_right) * 0.09
                        steer_angle = np.clip(steer_angle, -0.6, 0.6)

                        for pt in good_new:
                            x, y = pt.ravel()
                            cv2.circle(debug_frame, (int(x), int(y)), 3, (0, 255, 0), -1)

                        bar_end = int(160 + (steer_angle * 100))
                        cv2.line(debug_frame, (160, 220), (bar_end, 220), (0, 0, 255), 10)
                        prev_pts = good_new.reshape(-1, 1, 2)
                    else:
                        prev_pts = cv2.goodFeaturesToTrack(curr_gray, mask=None, **feature_params)
                else:
                    prev_pts = cv2.goodFeaturesToTrack(curr_gray, mask=None, **feature_params)
            else:
                prev_pts = cv2.goodFeaturesToTrack(curr_gray, mask=None, **feature_params)

            for j in steer_j:
                p.setJointMotorControl2(car_id, j, p.POSITION_CONTROL, targetPosition=steer_angle)

            cv2.imshow("Lucas-Kanade Tracking (From Scratch)", debug_frame)
            prev_gray = curr_gray.copy()
            p.stepSimulation()

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        p.disconnect()
