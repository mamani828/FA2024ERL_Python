import pybullet as p
import numpy as np

BOX_MASS = 50

WALL_MASS = 0  # Mass of 0 to make the wall static

class Robot:
    """
    Robot class for PyBullet simulation. Currently initializes
    a racecar urdf at (0, 0, 0.5). This ensures that the racecar
    is in the positive z-axis, and doesn't fall through the ground.
    """
    def __init__(self, config):
        # Constants post initialization
        self.steering_angle = config['steering_angle']
        self.robot_id = None
        self.wheel_indices = []
        self.steering_indices = []

        # Variable
        self.force = config['force']
        self.max_velocity = config['max_velocity']
        

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
        steer_angle = self.steering_angle
            
        if key == "W":
            # Apply forward force to all wheels
            for wheel in self.wheel_indices:
                p.setJointMotorControl2(self.robot_id, wheel, p.VELOCITY_CONTROL,
                                        targetVelocity=self.max_velocity, force=self.force)
        elif key == "S":
            # Apply backward force to all wheels
            for wheel in self.wheel_indices:
                p.setJointMotorControl2(self.robot_id, wheel, p.VELOCITY_CONTROL,
                                        targetVelocity=-self.max_velocity, force=self.force)
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

    def stop_motion(self):
        """
        Stops the robot's movement.
        """
        for wheel in self.wheel_indices:
            p.setJointMotorControl2(self.robot_id, wheel, p.VELOCITY_CONTROL,
                                    targetVelocity=0, force=0)
        
    def stop_angle(self):
        """
        Resets the angle to zero.
        """
        for steering in self.steering_indices:
            p.setJointMotorControl2(self.robot_id, steering, p.POSITION_CONTROL,
                                    targetPosition=0)

    def get_yaw(self):
        """
        Returns the yaw (orientation around the z-axis) of the robot.
        The robot's orientation is provided as a quaternion and we extract
        the yaw angle from it.
        """
        # Get the robot's current orientation as a quaternion (w, x, y, z)
        _, robot_orn = p.getBasePositionAndOrientation(self.robot_id)
        # Convert the quaternion to Euler angles (roll, pitch, yaw)
        euler_angles = p.getEulerFromQuaternion(robot_orn)
        yaw = euler_angles[2]  # Extract yaw (z-axis rotation)
        return yaw

    def get_pos(self):
        return p.getBasePositionAndOrientation(self.robot_id)[0]

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
        allowed_keys = {"max_velocity", "force"}
        
        self.__dict__.update((k, v) for k, v in kwargs.items() if k in allowed_keys)

        for k in kwargs.keys():
            if k not in allowed_keys:
                print("{} is not allowed to be updated".format(repr(k)))

    def __call__(self):
        # Return the robot_id when the class instance is called
        return self.robot_id

class CubeCreator:
    def __init__(self):
        self.cube_count = 0
        self.button_id = p.addUserDebugParameter("Create Cube", 1, 0, 1)
        self.prev_button_state = p.readUserDebugParameter(self.button_id)
        self.create_info_text()

    def create_info_text(self):
        self.info_text_id = p.addUserDebugText(
            text="Cubes created: 0",
            textPosition=[0, 0, 3],
            textColorRGB=[1, 1, 1],
            textSize=1.5
        )

    def update_info_text(self, x, y):
            p.addUserDebugText(
                text=f"Cubes created: {self.cube_count}",
                textPosition=[x, y, 3],
                textColorRGB=[1, 1, 1],
                textSize=1.5,
                replaceItemUniqueId=self.info_text_id
            )

    def update_cube_button(self, robot_pos):
        current_button_state = p.readUserDebugParameter(self.button_id)
        if current_button_state != self.prev_button_state:
            self.create_new_cube()
            self.cube_count += 1
            self.update_info_text(robot_pos[0], robot_pos[1])
            self.prev_button_state = current_button_state

    @staticmethod
    def create_new_cube():
        cube_size = 0.3
        x = np.random.normal(0, 4)
        y = np.random.normal(0, 4)
        position = [x, y, cube_size/2]
        orientation = p.getQuaternionFromEuler([0, 0, 0])
        color = list(np.random.uniform(0, 1, 3)) + [1]
        visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX,
                                        halfExtents=[cube_size/2]*3,
                                        rgbaColor=color)

        collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                halfExtents=[cube_size/2]*3)

        p.createMultiBody(baseMass=1, baseVisualShapeIndex=visual_shape,
                        baseCollisionShapeIndex=collision_shape,
                        basePosition=position,
                        baseOrientation=orientation)

    @staticmethod
    def create_wall(position, WALL_SIZE, WALL_COLOR):
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