import pybullet as p


class Robot:
    def __init__(self):
        self.robot_id = 1
        self.loading_robot()

    def loading_robot(self):
        # Set initial coordinates and orientation for r2d2
        racecar_coordinates = [0, 0, 0.5]  # Make sure it's on level ground
        racecar_orientation = [0, 0, 0]  # Neutral orientation (Euler Angles)
        initial_racecar_orientation = p.getQuaternionFromEuler(racecar_orientation)  # Quaternions
        racecar = p.loadURDF("racecar/racecar.urdf", racecar_coordinates,
                             initial_racecar_orientation, useFixedBase=False)
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