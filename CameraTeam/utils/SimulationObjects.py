import pybullet as p

BOX_MASS = 50


class Robot:
    """
    Robot class for PyBullet simulation. Currently initializes
    a racecar urdf at (0, 0, 0.5). This ensures that the racecar
    is in the positive z-axis, and doesn't fall through the ground.
    """
    def __init__(self):
        self.robot_id = None
        self.load_robot()

    def load_robot(self):
        """
        Loads the racecar.urdf file and sets the returned ID to
        self.robot_id.
        """
        self.robot_id = 1
        x =self.loading_robot()
       
    def loading_robot(self):
        # Set initial coordinates and orientation for r2d2
        racecar_coordinates = [0, 0, 0.5]  # Make sure it's on level ground
        racecar_orientation = [0, 0, 0]  # Neutral orientation (Euler Angles)
        initial_racecar_orientation = p.getQuaternionFromEuler(racecar_orientation)  # Quaternions
        self.robot_id = p.loadURDF("racecar/racecar.urdf", racecar_coordinates,
                             initial_racecar_orientation, useFixedBase=False)
        self.robot_id = racecar  # Set robot_id to id returned by loadURDF
    def __call__(self):
        # Return the robot_id when the class instance is called
        return self.robot_id
     


class Object:
    """
    Object class that initializes a box.
    """
    def __init__(self, coordinates, color):
        """
        Instance variables such as color and coordinates that
        set the color and coordinates of the box.

        Args:
            coordinates: (x, y, z) coordinates of the box.
            color: color of the box.
        """
        self.color = color
        self.coordinates = coordinates
        self.object_id = self.loading_box()  # Storing the object's ID

    def loading_box(self):
        """
        Loads the box into the simulation with the given instance
        variables. The box is g

        Returns:
            Multibody: Multibody object 
        """
        half_extents = [0.3, 0.3, 0.3]
        box_coordinates = self.coordinates
        box_orientation = [0, 0, 0]
        initial_box_orientation = p.getQuaternionFromEuler(box_orientation)

        box_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=half_extents)
        box_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=half_extents, rgbaColor=self.color)

        # Create a multibody and return its ID
        return p.createMultiBody(baseMass=BOX_MASS, baseCollisionShapeIndex=box_collision_shape,
                                 baseVisualShapeIndex=box_visual_shape,
                                 basePosition=box_coordinates,
                                 baseOrientation=initial_box_orientation)