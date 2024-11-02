import pybullet as p
import pybullet_data
import numpy as np

class Camera:
    def __init__(self, target_robot, config):
        self.target_robot = target_robot
        self.cam_distance = 1.5
        self.yaw = config['yaw']
        self.pitch = config['pitch']  
        self.roll = config['roll']

    def update_view_matrix(self):
        # Get the robot's position and orientation  
        robot_pos, _ = p.getBasePositionAndOrientation(self.target_robot.robot_id)
        
        # Camera is behind and above robot
        camera_pos = [robot_pos[0] - self.cam_distance, robot_pos[1], robot_pos[2] + 0.5]
        view_matrix = p.computeViewMatrix(camera_pos, robot_pos, [0, 0, 1])
        
        return view_matrix

class Lidar:
    pass