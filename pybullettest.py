import pybullet as p
import pybullet_data
import time
import numpy as np


''' Class that creates the enviorment for the simulation, the idea is to have all simulation 
functionality in this class and/or in conjuction with other classes. '''
class PyBulletEnvironment:
    def __init__(self):
        ''' Standard pybullet setup, setting up the simulation environment, setting up the gravity, textbox, ...'''
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.plane_id = p.loadURDF("plane.urdf")
        self.create_button()
        self.walls = Walls()
        self.robot = Robot()
        self.create_info_text()
        self.cube_count = 0

    def create_button(self):
        self.button_id = p.addUserDebugParameter("Create Cube", 1, 0, 1)
        self.prev_button_state = p.readUserDebugParameter(self.button_id)

    def run_simulation(self):
        while True:
            p.stepSimulation()
            self.check_button()
            time.sleep(1./240.)
    '''Function that checks the button state, if the button is pressed, it creates a new cube.'''
    def check_button(self):
        current_button_state = p.readUserDebugParameter(self.button_id)
        if current_button_state != self.prev_button_state:
            self.create_new_cube()
            self.cube_count += 1
            self.update_info_text()
            self.prev_button_state = current_button_state
    def create_info_text(self):
        self.info_text_id = p.addUserDebugText(
            text="Cubes created: 0",
            textPosition=[0, 0, 3],
            textColorRGB=[1, 1, 1],
            textSize=1.5
        )
        '''use the count or any other custom function to update your text'''
    def update_info_text(self):
            p.addUserDebugText(
                text=f"Cubes created: {self.cube_count}",
                textPosition=[0, 0, 3],
                textColorRGB=[1, 1, 1],
                textSize=1.5,
                replaceItemUniqueId=self.info_text_id
            )

    @staticmethod
    def create_new_cube():
        cube_size = 0.3
        x = np.random.normal(0, 4)
        y = np.random.normal(0, 4)
        position = [x, y, cube_size/2]
        orientation = p.getQuaternionFromEuler([0, 0, 0])
        color = list(np.random.uniform(0, 1, 3)) + [1]  # Random RGB + Alpha
        visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=[cube_size/2]*3, rgbaColor=color)
        collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=[cube_size/2]*3)
        p.createMultiBody(baseMass=1, baseVisualShapeIndex=visual_shape, baseCollisionShapeIndex=collision_shape, basePosition=position, baseOrientation=orientation)

'''Walls class is about creating the walls in the sim, funciton regarding the characterstics
of the walls is designed and implemented in here.'''
class Walls:
    def __init__(self):
        self.create_default_walls()

    @staticmethod
    def create_wall(position, size):
        visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=size)
        collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=size)
        p.createMultiBody(baseMass=0, baseVisualShapeIndex=visual_shape, baseCollisionShapeIndex=collision_shape, basePosition=position)

    def create_default_walls(self, wall_height=0.5, wall_thickness=0.1):
        # Outer walls
        self.create_wall([0, 5, wall_height/2], [5, wall_thickness, wall_height/2])  # North wall
        self.create_wall([0, -5, wall_height/2], [5, wall_thickness, wall_height/2])  # South wall
        self.create_wall([5, 0, wall_height/2], [wall_thickness, 5, wall_height/2])  # East wall
        self.create_wall([-5, 0, wall_height/2], [wall_thickness, 5, wall_height/2])  # West wall
        # Inner walls
        self.create_wall([-2.5, 2.5, wall_height/2], [2.5, wall_thickness, wall_height/2])
        self.create_wall([2.5, -2.5, wall_height/2], [2.5, wall_thickness, wall_height/2])
        self.create_wall([0, 0, wall_height/2], [wall_thickness, 2.5, wall_height/2])

''' Robot class is creating the robot, for now it has no functionality.'''
class Robot:
    def __init__(self):
        self.create_robot()

    @staticmethod
    def create_robot():
        robot_size = 0.3
        start_pos = [-4, -4, robot_size/2]
        start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        p.loadURDF("cube.urdf", start_pos, start_orientation, globalScaling=robot_size)

if __name__ == "__main__":
    env = PyBulletEnvironment()
    try:
        env.run_simulation()
    except KeyboardInterrupt:
        p.disconnect()
        print("Simulation stopped by user.")