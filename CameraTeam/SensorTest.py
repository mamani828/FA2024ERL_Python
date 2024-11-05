import pybullet as p
import pybullet_data
import numpy as np
import time
import os
import yaml
import sys
from Sensors import Camera, Lidar

SENSOR_CONFIG_PATH = os.path.join(os.path.dirname(__file__),
                                  'config/sensors.yaml')


def open_yaml(config_name):
    try:
        with open(SENSOR_CONFIG_PATH, 'r') as file:
            return yaml.safe_load(file)[config_name]
    except FileNotFoundError:
        print("Config file not found!")
        sys.exit(0)


class PybulletEnvironment:
    def __init__(self):
        self.physics_client = p.connect(p.GUI)  # Connect to Physx simulator
        p.setGravity(0, 0, -9.81)  # Setting gravity value for the simulation
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        coordinates = [0, 0, 0]  # initial coordinates and orientation for the plane
        orientation = [0, 0, 0]  # Euler Angles Roll, Pitch, Yaw
        initial_orientation = p.getQuaternionFromEuler(orientation)  # Converting to Quaternion
        self.plane_id = p.loadURDF("plane.urdf", coordinates, initial_orientation)

    def run_simulation(self, *args):
        while True:
            p.stepSimulation()
            time.sleep(1 / 240)
            # TODO: args is a tuple of sensors. Iterate through this tuple
            #       and update each sensor

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
        racecar = p.loadURDF("racecar/racecar.urdf", racecar_coordinates,
                             initial_racecar_orientation, useFixedBase=True)
        self.robot_id = racecar  # Set robot_id to id returned by loadURDF


class Object:
    def __init__(self, coordinates, color):
        self.color = color
        self.coordinates = coordinates
        self.object_id = self.loading_box()  # Storing the object's ID

    def loading_box(self):
        half_extents = [0.5, 0.5, 0.5]
        box_coordinates = self.coordinates
        box_orientation = [0, 0, 0]
        initial_box_orientation = p.getQuaternionFromEuler(box_orientation)

        box_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=half_extents)
        box_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=half_extents, rgbaColor=self.color)

        # Create a multibody and return its ID
        return p.createMultiBody(baseMass=50, baseCollisionShapeIndex=box_collision_shape,
                                 baseVisualShapeIndex=box_visual_shape,
                                 basePosition=box_coordinates,
                                 baseOrientation=initial_box_orientation)
"""
@target_object params
first set of [] coordinates of box & second set of [] are the colors of the box
"""

if __name__ == "__main__":
    sim = PybulletEnvironment()
    robot = Robot()
    target_object = Object([2, 2, 2], [1, 0, 0, 1])  # Spawning box after initializing PybulletSim

    camera_config = open_yaml("camera_configs")
    camera = Camera(robot, camera_config)

    #  lidar_config = open_yaml("lidar_configs")
    #  lidar = Lidar(robot, lidar_config)

    for _ in range(100000):
        p.stepSimulation()
        time.sleep(1 / 240)  # Slow down the simulation to real time

        view_matrix = camera.update_view_matrix()
        p.getCameraImage(1920, 1080, viewMatrix=view_matrix)
        # Disconnect when done
    sim.disconnect()
