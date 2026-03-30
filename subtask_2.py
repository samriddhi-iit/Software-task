import cv2
import numpy as np
import tempfile
import pathlib
import pybullet as p
import pybullet_data
import time



def make_obstacle_texture(size: int = 128, tile: int = 10) -> str:
    img    = np.zeros((size, size, 3), dtype=np.uint8)
    black  = np.array([0, 0, 0], dtype=np.uint8)
    yellow = np.array([0, 215, 255], dtype=np.uint8)

    for row in range(size):
        for col in range(size):
            img[row, col] = black if (row // tile + col // tile) % 2 == 0 else yellow

    tex_path = pathlib.Path(tempfile.gettempdir()) / "obs_texture.png"
    ok = cv2.imwrite(str(tex_path), img)
    if not ok:
        raise RuntimeError(f"cv2.imwrite failed writing texture to: {tex_path}")

    print(f"[Texture] Saved yellow/black checkerboard → {tex_path}")
    return str(tex_path)


# =============================================================================
# ROAD + OBSTACLES
# =============================================================================

def create_road_and_obstacles():

    tex_path = make_obstacle_texture(size=128, tile=10)
    tex_id   = p.loadTexture(tex_path)

    rv = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[16.66, 1.16, 0.01],
        rgbaColor=[0.15, 0.15, 0.15, 1]
    )
    rc = p.createCollisionShape(p.GEOM_BOX, halfExtents=[16.66, 1.16, 0.01])
    p.createMultiBody(0, rc, rv, [16.66, 0, 0.01])
    

    for x in np.arange(0, 34, 0.4):
        lv = p.createVisualShape(
            p.GEOM_BOX, halfExtents=[0.12, 0.03, 0.01],
            rgbaColor=[1, 1, 1, 1]
        )
        p.createMultiBody(0, -1, lv, [x,  0.0,  0.02])
        p.createMultiBody(0, -1, lv, [x,  0.85, 0.02])
        p.createMultiBody(0, -1, lv, [x, -0.85, 0.02])

    obs_extents = [0.25, 0.45, 0.35]

    for i, x in enumerate(range(6, 30, 6)):

        y = 0.38 if i % 2 == 0 else -0.38

        ov = p.createVisualShape(
            p.GEOM_BOX, halfExtents=obs_extents,
            rgbaColor=[1, 1, 1, 1]
        )

        oc = p.createCollisionShape(p.GEOM_BOX, halfExtents=obs_extents)

        obs_id = p.createMultiBody(10, oc, ov, [x, y, 0.35])

        p.changeVisualShape(obs_id, -1, textureUniqueId=tex_id)

        print(f"[Obstacle {i}] x={x}m  y={y:+.2f}m  texture applied")

    wv = p.createVisualShape(
        p.GEOM_BOX, halfExtents=[0.5, 1.16, 0.5],
        rgbaColor=[0.1, 0.3, 0.9, 1]
    )

    wc = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5, 1.16, 0.5])

    p.createMultiBody(0, wc, wv, [31.66, 0, 0.5])

    print("[End wall] placed at x=31.66 m")


# =============================================================================
# CAR
# =============================================================================

def create_car(start_pos=None, start_orn=None, global_scaling=1.8):

    if start_pos is None:
        start_pos = [0, 0, 0.25]

    if start_orn is None:
        start_orn = p.getQuaternionFromEuler([0, 0, 0])

    car_id = p.loadURDF(
        "racecar/racecar.urdf",
        start_pos,
        start_orn,
        globalScaling=global_scaling
    )

    p.changeDynamics(car_id, -1, ccdSweptSphereRadius=0.1)

    steering_joints = []
    motor_joints = []

    for i in range(p.getNumJoints(car_id)):

        name = p.getJointInfo(car_id, i)[1].decode("utf-8")

        if "steer" in name.lower():
            steering_joints.append(i)

        elif "wheel" in name.lower():
            motor_joints.append(i)

    print(
        f"[Car] body_id={car_id}  steering_joints={steering_joints}  motor_joints={motor_joints}"
    )

    return car_id, steering_joints, motor_joints


# =============================================================================
# SETUP
# =============================================================================

def setup_simulation(dt=1.0 / 60.0, settle_frames=60, gui=True):

    mode = p.GUI if gui else p.DIRECT

    if gui:
        p.connect(p.GUI, options="--opengl2 --tinyRenderer")
    else:
        p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    p.setGravity(0, 0, -9.81)

    p.setTimeStep(dt)

    p.loadURDF("plane.urdf")

    create_road_and_obstacles()

    car_id, steering_joints, motor_joints = create_car()

    print(f"[Setup] Settling suspension for {settle_frames} frames …")

    for _ in range(settle_frames):

        p.stepSimulation()

        time.sleep(dt)

    print("[Setup] Ready.")

    return car_id, steering_joints, motor_joints


# =============================================================================
# DEMO
# =============================================================================

if __name__ == "__main__":

    car_id, steer_j, motor_j = setup_simulation(gui=True)

    print("\nSimulation running. Close the PyBullet window or press Ctrl+C to exit.")

    dt = 1.0 / 60.0

    try:

        while True:

            for j in motor_j:

                p.setJointMotorControl2(
                    car_id,
                    j,
                    p.VELOCITY_CONTROL,
                    targetVelocity=5.0,
                    force=800
                )

            p.stepSimulation()

            time.sleep(dt)

    except KeyboardInterrupt:

        pass

    finally:

        try:
            p.disconnect()
        except Exception:
            pass

        print("Simulation ended.")
