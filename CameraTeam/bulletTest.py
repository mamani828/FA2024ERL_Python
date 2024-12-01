import pybullet as p
import pybullet_data
import time
import threading

def camera_update_thread(simulation_rate, camera_rate):
    """
    Updates the camera view at half the simulation FPS.
    """
    while True:
        # Capture and update the camera view
        view_matrix = p.computeViewMatrix(
            cameraEyePosition=[1, 1, 1],
            cameraTargetPosition=[0, 0, 0],
            cameraUpVector=[0, 0, 1],
        )
        projection_matrix = p.computeProjectionMatrixFOV(
            fov=60, aspect=1.0, nearVal=0.1, farVal=100
        )
        _, _, _, _, camera_image = p.getCameraImage(
            width=320, height=240, viewMatrix=view_matrix, projectionMatrix=projection_matrix
        )
        # Simulate half-rate update
        time.sleep(1 / camera_rate)

def pybullet_simulation(simulation_rate, camera_rate):
    """
    Main PyBullet simulation loop.
    """
    # Initialize PyBullet and the environment
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    plane_id = p.loadURDF("plane.urdf")
    cube_id = p.loadURDF("r2d2.urdf", [0, 0, 1])

    # Start camera update thread
    camera_thread = threading.Thread(target=camera_update_thread, args=(simulation_rate, camera_rate))
    camera_thread.daemon = True
    camera_thread.start()

    # Simulation loop
    simulation_dt = 1.0 / simulation_rate
    while True:
        p.stepSimulation()
        time.sleep(simulation_dt)

if __name__ == "__main__":
    SIMULATION_FPS = 240  # Simulation frames per second
    CAMERA_FPS = SIMULATION_FPS / 6  # Camera updates at half the simulation FPS
    pybullet_simulation(SIMULATION_FPS, CAMERA_FPS)
