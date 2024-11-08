import pybullet as p
import pybullet_data
import numpy as np
import math
from abc import ABC, abstractmethod


class Sensor(ABC):
    """
    Sensor abstract base class. ABC acts as a blueprint class,
    and enforces that all subclasses must have certain methods
    implemented. If those classes are not implemented, the code
    will not compile.

    Note: Feel free to add or remove methods in this class. Also,
    do not use this class until later.

    Args:
        ABC (Class):
    """
    @abstractmethod
    def __init__(self):
        """
        Every sensor class must have initializing method.
        """
        pass

    @abstractmethod
    def setup(self):
        """
        Every sensor class must have a setup method.
        """
        pass

    @abstractmethod
    def get_data(self):
        """
        Every sensor class must have a method to obtain data.
        """
        pass

    @abstractmethod
    def update_sensor(self):
        """
        Every sensor that needs an update will have an update method.
        """

    @abstractmethod
    def attach_to_car(self):
        """
        Every sensor must be attached to the car.
        """
        pass


class Camera:
    """
        Camera sensor that is attached to the car. Provides visual feedback
        as to what the car currently sees.
    """
    def __init__(self, target_robot: int, config: dict) -> None:
        """
        Initializes the robot camera.

        Args:
            target_robot (int): car id.
            config (dict):
                - camera_distance: Constant int to alter camera location
                - yaw: y-axis rotation
                - pitch: x-axis rotation
                - roll: z-axis rotation
        """
        self.CAMERA_DISTANCE = config['camera_distance']
        self.YAW = config['yaw']
        self.PITCH = config['pitch']
        self.ROLL = config['roll']

        # Gets the car ID
        self.target_robot = target_robot

    def update_sensor(self) -> list:
        """
        Computes the current view_matrix of the camera.

        Args:
            None.
        Returns:
            view_matrix:
        Raises:
            None.
        """
        # Get the robot's position and orientation
        robot_pos, _ = p.getBasePositionAndOrientation(self.target_robot.robot_id)

        # Camera is behind and above robot
        camera_pos = [robot_pos[0] - self.CAMERA_DISTANCE, robot_pos[1], robot_pos[2] + 0.5]
        view_matrix = p.computeViewMatrix(camera_pos, robot_pos, [0, 0, 1])

        return view_matrix


#  Note: Imported from main GitHub. Modify as needed.
class Lidar:
    """
    Light radar class. Uses Pybullet's rayTestBatch.
    """
    def __init__(self, target_robot: object, config: dict) -> None:
        """
        Initializes the Lidar class by setting variables.

        Args:
            target_robot(object): PyBullet Robot.
            config(dict):
                lidar_joints: Integer joint of the robot
                hit_color: RGB when ray hits an object
                miss_color: RGB when ray does not hit an object
                num_rays: Integer number of rays
                lidar_angle1: Integer starting angle
                lidar_angle2: Integer ending angle
                ray_start_len: Integer ray starting length
                ray_len: Integer ray length
        Returns:
            None.
        Raises:
            None.
        """
        #  Constants
        self.LIDAR_JOINTS = config['lidar_joints']
        self.HIT_COLOR = config['hit_color']
        self.MISS_COLOR = config['miss_color']

        #  Default from config, GUI overrides
        self.num_rays = config['num_rays']
        self.start_angle = config['lidar_angle1']
        self.end_angle = config['lidar_angle2']
        self.start_len = config['ray_start_len']
        self.ray_len = config['ray_len']


        self.target_robot = target_robot
        self.car_id = target_robot.robot_id

    def setup(self):
        """
        Default setup for when lidar object is just built
        """
        a = self.start_angle*(math.pi/180)
        b = (self.end_angle - self.start_angle)*(math.pi/180)

        ray_from, ray_to = [], []
        for i in range(self.num_rays):
            theta = float(a) + (float(b) * (float(i)/self.num_rays))
            x1 = self.start_len*math.sin(theta)
            y1 = self.start_len*math.cos(theta)
            z1 = 0

            x2 = self.ray_len*math.sin(theta)
            y2 = self.ray_len*math.cos(theta)
            z2 = 0

            ray_from.append([x1, y1, z1])
            ray_to.append([x2, y2, z2])

        self.ray_from = ray_from
        self.ray_to = ray_to

        ray_ids = []
        for i in range(self.num_rays):
            ray_ids.append(
                    p.addUserDebugLine(
                            self.ray_from[i],
                            self.ray_to[i],
                            self.MISS_COLOR,
                            parentObjectUniqueId=self.car_id,
                            parentLinkIndex=self.LIDAR_JOINTS
                        )
                    )
        self.ray_ids = ray_ids

    def retrieve_data(self, common=True, robot_state=None) -> tuple:
        """
        In most mobile robot applications, each ray (or lidar)
        gives two information: 1. range, 2. bearing (w.r.t car's
        yaw). Range is how far the ray reached - if it hits something
        than it should be less than it's max length. Bearing tells
        us where this scan occurs in the map w.r.t the car's frame.

        However, PyBullet has nicely given us the coordinates in
        the world frame of obstacles that a ray hits.

        Hence, this function will implement both cases. Set
        common=False for the second case (since the first case
        is more common in mobile robots problems)

        Args:
            robot_state: The car's current state (x, y, yaw). Some sensors
                       need this, some don't. It is included
                       for consistency of the API.
            common: For API consistency, same as `robot_state`.
        Returns:
            rays_data: A numpy array needed to simulate LiDAR.
            coords: Coordinate (x, y) of hit points in world
                    coordinate. For a ray that does not hit an object,
                    the value is None.
            dists: Distance to hit object for every ray. For a ray that
                   does not hit an object, the value is self.RAY_LEN

        """
        num_threads = 0
        rays_data = p.rayTestBatch(
                    self.ray_from,
                    self.ray_to,
                    num_threads,
                    parentObjectUniqueId=self.car_id,
                    parentLinkIndex=self.LIDAR_JOINTS
                )

        # Convert `results` to numpy array to leverage Numpy's speed.
        rays_data = np.array(rays_data, dtype=object)

        # Get the rays which does not hit an object
        no_hit = np.where(rays_data[:, 2] == 1.)[0]

        # Convert `coords` to numpy array to leverage Numpy's speed.
        # `coords` will be of shape (NUM_RAYS, 3) and coords[i] contains
        # [x, y, z] coordinates in world coordinate, for each rays.
        coords = np.array(rays_data[:, 3], dtype=object)  # shape: (NUM_RAYS, )
        coords = np.stack(coords).astype(np.float32)      # shape: (NUM_RAYS, 3)
        coords = coords[:, :2]  # We only need (x, y) coords.

        # Get the distances
        x, y, yaw = robot_state
        dists = np.sqrt((coords[:, 0] - x)**2 + (coords[:, 1] - y)**2)  # (NUM_RAYS, )

        # Set distances to self.RAY_LEN for rays that doesn't hit an object.
        dists[no_hit] = self.RAY_LEN

        # Get bearing data
        angle = np.arctan2(coords[:, 1] - y, coords[:, 0] - x)
        angle *= 180.0/np.pi
        bearings = angle - yaw
        bearings = np.where(bearings < 0, bearings + 360, bearings)

        if common:
            return rays_data, dists, bearings

        # set coord to None for rays with no hit.
        coords[no_hit] = None  # Set coord to None for rays with not hit.
        return rays_data, dists, coords

    def gui_change_parameter(self, **kwargs) -> None:
        """
        Changes the parameters in Lidar class through the GUI. This will
        affect the next call to setup as the values have changed.

        Args:
            kwargs(dict): Variables to be changed.
        Returns:
            None.
        Raises:
            None.
        """
        allowed_keys = {"num_rays", "start_angle", "end_angle", "start_len",
                        "ray_len"}
        
        self.__dict__.update((k, v) for k, v in kwargs.items() if k in allowed_keys)

        for k in kwargs.keys():
            if k not in allowed_keys:
                print("{} is not allowed to be updated".format(repr(k)))