import pybullet as p


BOX_MASS = 50
FORCE = 1
STEERING_ANGLE = 0.5
MAX_VELOCITY = 10
WALL_MASS = 0  # Mass of 0 to make the wall static

class Robot:
    """
    Robot class for PyBullet simulation. Currently initializes
    a racecar urdf at (0, 0, 0.5). This ensures that the racecar
    is in the positive z-axis, and doesn't fall through the ground.
    """
    def __init__(self):
        self.robot_id = None
        self.wheel_indices = []
        self.steering_indices = []
        self.load_robot()

    def load_robot(self):
        """
        Loads the racecar.urdf file and sets the returned ID to
        self.robot_id.
        """
        racecar_coordinates = [0, 0, 0.2]  # Make sure it's on level ground
        racecar_orientation = [0, 0, 0]  # Neutral orientation (Euler Angles)
        initial_racecar_orientation = p.getQuaternionFromEuler(racecar_orientation)  # Quaternions
        self.robot_id = p.loadURDF("racecar/racecar.urdf", racecar_coordinates,
                             initial_racecar_orientation, useFixedBase=False)

        num_joints = p.getNumJoints(self.robot_id)
        for joint_index in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, joint_index)
            joint_name = joint_info[1].decode("utf-8")
            if "wheel" in joint_name:
                self.wheel_indices.append(joint_index)
            elif "steering" in joint_name:
                self.steering_indices.append(joint_index)
    
    def move(self, key):
        """
        Moves the robot based on key presses (W, A, S, D).
        W: Forward
        S: Backward
        A: Turn Left
        D: Turn Right
        """
        force = FORCE
        steer_angle = STEERING_ANGLE
            
        if key == "W":
            # Apply forward force to all wheels
            for wheel in self.wheel_indices:
                p.setJointMotorControl2(self.robot_id, wheel, p.VELOCITY_CONTROL,
                                        targetVelocity=MAX_VELOCITY, force=force)
        elif key == "S":
            # Apply backward force to all wheels
            for wheel in self.wheel_indices:
                p.setJointMotorControl2(self.robot_id, wheel, p.VELOCITY_CONTROL,
                                        targetVelocity=-MAX_VELOCITY, force=force)
        elif key == "A":
            # Turn left by adjusting steering joints
            for steering in self.steering_indices:
                p.setJointMotorControl2(self.robot_id, steering, p.POSITION_CONTROL,
                                        targetPosition=steer_angle)
        elif key == "D":
            # Turn right by adjusting steering joints
            for steering in self.steering_indices:
                p.setJointMotorControl2(self.robot_id, steering, p.POSITION_CONTROL,
                                        targetPosition=-steer_angle)

    def stop(self):
        """
        Stops the robot's movement.
        """
        for wheel in self.wheel_indices:
            p.setJointMotorControl2(self.robot_id, wheel, p.VELOCITY_CONTROL,
                                    targetVelocity=0, force=0)
        for steering in self.steering_indices:
            p.setJointMotorControl2(self.robot_id, steering, p.POSITION_CONTROL,
                                    targetPosition=0)

    def __call__(self):
        # Return the robot_id when the class instance is called
        return self.robot_id
    def get_yaw(self):
        """
        Returns the yaw (orientation around the z-axis) of the robot.
        The robot's orientation is provided as a quaternion and we extract
        the yaw angle from it.
        """
        # Get the robot's current orientation as a quaternion (w, x, y, z)
        robot_pos, robot_orn = p.getBasePositionAndOrientation(self.robot_id)
        # Convert the quaternion to Euler angles (roll, pitch, yaw)
        euler_angles = p.getEulerFromQuaternion(robot_orn)
        yaw = euler_angles[2]  # Extract yaw (z-axis rotation)
        return yaw


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
        #self.object_id = self.loading_box()  # Storing the object's ID

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
    def create_wall(self, position, WALL_SIZE, WALL_COLOR):
        """
        Create a wall in the PyBullet environment.
        
        :param position: The position (x, y, z) where the wall will be placed.
        :param size: The size (length, width, height) of the wall. Defaults to WALL_SIZE.
        :param color: The color of the wall. Defaults to WALL_COLOR.
        :return: The ID of the wall object created in the simulation.
        """
        size = WALL_SIZE
        color = WALL_COLOR
        # Create a box collision shape for the wall
        
        wall_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[size[0]/2, size[1]/2, size[2]/2])
        
        # Create a box visual shape for the wall
        wall_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[size[0]/2, size[1]/2, size[2]/2], rgbaColor=color)
        
        # Create the wall as a static body (no movement)
        wall_id = p.createMultiBody(baseMass=WALL_MASS, baseCollisionShapeIndex=wall_collision_shape, 
                                    baseVisualShapeIndex=wall_visual_shape, basePosition=position)
        
        return wall_id