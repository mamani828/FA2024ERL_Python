import pybullet as p
import pybullet_data
import numpy as np
import time
import os
from Sensors import Camera, Lidar

class PybulletSim:
    def __init__(self):
        self.physics_client = p.connect(p.GUI)  # Connecting to the Physics simulator
        p.setGravity(0, 0, -9.81)  # Setting the gravity value for the simulation
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        coordinates = [0, 0, 0]  # initial coordinates and orientation for the plane
        orientation = [0, 0, 0]  # Euler Angles Roll, Pitch, Yaw
        initial_orientation = p.getQuaternionFromEuler(orientation)  # Converting to Quaternion
        self.plane_id = p.loadURDF("plane.urdf", coordinates, initial_orientation)

    def disconnect(self):
        p.disconnect(self.physics_client)


class Robot:
    def __init__(self):
        self.robot_id = None
        self.loading_robot()

    def loading_robot(self):
        # Set initial coordinates and orientation for r2d2
        racecar_coordinates = [0, 0, 0.5]  # Make sure it's on level ground
        racecar_orientation = [0, 0, 0]  # Neutral orientation (Euler Angles)
        initial_racecar_orientation = p.getQuaternionFromEuler(racecar_orientation)  # Quaternions
        racecar = p.loadURDF("racecar/racecar.urdf", racecar_coordinates, initial_racecar_orientation)
        self.robot_id = racecar # Set robot_id to id returned by loadURDF

if __name__ == "__main__":
    sim = PybulletSim()
    robot = Robot()
    camera = Camera(robot)

    for _ in range(100000):
        p.stepSimulation()
        time.sleep(1 / 240)  # Slow down the simulation to real time

        view_matrix = camera.update_view_matrix()
        p.getCameraImage(1920, 1080, viewMatrix=view_matrix)
        # Disconnect when done
    sim.disconnect()
