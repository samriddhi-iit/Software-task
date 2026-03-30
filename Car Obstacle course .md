# Car Obstacle course

- Objective
    
    To simulate a **self-driving car environment** using **PyBullet**, where a vehicle navigates a structured road with static obstacles, enabling testing of motion control, perception, and autonomous driving algorithms.
    
- Input
    
    ### 1. Simulation Parameters
    
    - Time step (`dt`)
    - Number of settling frames
    - GUI mode (enabled/disabled)
    
    ### 2. Environment Configuration
    
    - Road dimensions
    - Obstacle positions and sizes
    - Texture parameters (checkerboard size, tile size)
    
    ### 3. Vehicle Parameters
    
    - Initial position and orientation
    - Scaling factor of the car model
    - Joint configurations (steering and motor joints)
    
    ### 4. Control Inputs
    
    - Target velocity for wheels
    - Steering angle (if implemented)
- Output
    
    ### 1. Visual Output
    
    - Real-time 3D simulation of:
        - Road and lane markings
        - Obstacles with textures
        - Moving vehicle
    
    ### 2. Simulation State Data
    
    - Vehicle position and orientation over time
    - Joint states (velocity, steering angles)
    
    ### 3. Event Feedback (Console)
    
    - Obstacle placement logs
    - Car joint configuration details
    - Simulation status messages
    
    ### 4. (Optional Extensions)
    
    - Collision detection results
    - Camera sensor images
    - Performance metrics (e.g., distance traveled, collisions)
    
- Setting up the setup and running the simulation
    
    I first tried setting up PyBullet inside a container, but the GUI would not render because the container could not connect to the host X11 server. 
    
    I then tried installing PyBullet directly on my local macOS machine, but the installation kept failing. 
    
    To move forward, I created a clean Python environment using **Miniforge** (conda) and installed the required dependencies there, which allowed me to run the simulation reliably.
    
- Capturing RGB frames from the car’s perspective
    
    Creating the “Eyes” of the autonomous [agent. Camera](http://agent.Camera) moves perfectly in sync with the car’s body.
    
    - Here is a list of all the technical components used and their purposes.
    
    | Operation | Technical Component | Purpose |
    | --- | --- | --- |
    | State Retrieval | p.getBasePositionAndOrientation | Gets the car's 3D coordinates (x,y,z) and rotation (quaternion) from the physics engine. |
    | Vector Math | p.getMatrixFromQuaternion | Converts rotation into a matrix to extract the Forward Vector, ensuring the camera "points" where the car "faces." |
    | Placement | camera_pos Calculation | Offsets the camera 0.5m forward and 0.3m up to clear the car's body and provide a clear view of the road. |
    | Orientation | p.computeViewMatrix | Defines the camera's "Eye," "Target," and "Up" vectors to create the view transformation. |
    | Lens Simulation | p.computeProjectionMatrixFOV | Sets a 90° FOV and 1.33 aspect ratio, mimicking a wide-angle lens to capture peripheral obstacles. |
    | Rendering | p.getCameraImage | Triggers the ER_TINY_RENDERER to convert 3D geometry into a 2D array of pixel data. |
    | Data Cleaning | np.reshape & Slicing | Transforms the flat pixel list into a (240,320,3) matrix and removes the Alpha channel for OpenCV compatibility. |
    
    **Syncs with Physics:** Hooks the camera to the car's current <X, Y, Z>position.
    
    **Solves Direction:** Uses a **Rotation Matrix** to compute the car's "Forward" vector, ensuring the camera always points where the car is heading.
    
    **Virtual Mounting:** Places the camera **0.5m ahead** and **0.3m up** to get a clean view without the car's hood blocking the frame.
    
    **Simulates Vision:** Applies a **90° wide-angle lens** to capture side obstacles, mimicking biological peripheral vision.
    
    **Generates Data:** Converts 3D simulation math into a **320x240 RGB image** that OpenCV can use to calculate motion.
    

---

### 4. Simulation Setup

### `setup_simulation(...)`

---

## Step 1: Connect physics engine

```
p.connect(p.GUIorp.DIRECT)
```

- GUI → visualization
- DIRECT → headless

---

## Step 2: Environment setup

```
p.setGravity(0,0,-9.81)
p.setTimeStep(dt)
p.loadURDF("plane.urdf")
```

---

## Step 3: Create world

```
create_road_and_obstacles()
create_car()
```

---

## Step 4: Settling phase

```
for_inrange(settle_frames):
p.stepSimulation()
```

- Lets suspension stabilize

---

## Output

```
returncar_id,steering_joints,motor_joints
```

---

## 5. Main Simulation Loop

## Inside `if __name__ == "__main__":`

---

## Step 1: Setup

```
car_id,steer_j,motor_j=setup_simulation()
```

---

## Step 2: Infinite loop

```
whileTrue:
```

---

## Step 3: Apply motor control

```
p.setJointMotorControl2(
car_id,
j,
p.VELOCITY_CONTROL,
targetVelocity=5.0,
force=800
)
```

- Drives wheels forward

---

## Step 4: Step simulation

```
p.stepSimulation()
```

- Advances physics engine

---

## Step 5: Timing

```
time.sleep(dt)
```

- Keeps real-time speed

---

## Step 6: Exit handling

```
exceptKeyboardInterrupt:
```

- Allows Ctrl+C exit

```
p.disconnect()
```

- Clean shutdown
