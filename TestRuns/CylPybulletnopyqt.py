import json
import sys
import os
import numpy as np
import pybullet as p
import pybullet_data
import time
from PyQt5.QtWidgets import QApplication, QWidget

class PybulletEnvironment:
    
    def __init__(self):
        super().__init__()
        
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.plane_id = p.loadURDF("plane.urdf")
        self.Cylinder = Cylinder()
        
        # initializing the PyQt GUI
        self.app = QApplication(sys.argv)
        self.ControlPanel = QWidget()
        self.ControlPanel.show()
        
    def update_info_text(self):
        p.addUserDebugText(
            text=f"velocities: {self.x_vel, self.y_vel}",
            textPosition=[0, 0, 3],
            textColorRGB=[1, 1, 1],
            textSize=1.5,
            replaceItemUniqueId=getattr(self, 'info_text_id', None)  # Ensure it exists before using
        )
        print(self.Cylinder.position)
    
    def getFrame(self):
        width, height, rgbImg, depthImg, segImg = p.getCameraImage(width=640, height=480)
        rgb_array = np.array(rgbImg)
        with open("frame.json", "w") as file:
            json.dump(rgb_array.tolist(), file)
            
    def run_simulation(self):
        Pid_X = PID(0)
        Pid_Y = PID(0)
        while True:
            p.stepSimulation()
            time.sleep(1./240.)
            self.Cylinder.updatePosition()
            self.ControlPanel.update()
            try:
                # Update values from the ControlPanel if available
                if hasattr(self.ControlPanel, 'get_values'):
                    values = self.ControlPanel.get_values()
                    self.x_vel = values.get('X_velocity', 0)
                    self.y_vel = values.get('Y_velocity', 0)
                    self.x_goalpos = values.get('X_goalpos', 0)
                    self.y_goalpos = values.get('Y_goalpos', 0)
                
                # Apply the velocities to the cylinder
                self.Cylinder.setVelocity(self.x_vel, self.y_vel)
                
            except AttributeError:
                print("ControlPanel does not have method 'get_values'") 
                continue
            
class Cylinder: 
    def __init__(self):
        cylinder_height = 2
        self.cylinder_id = p.loadURDF("cylinder.urdf", basePosition=[0, 0, cylinder_height / 2])
        self.position = self.getPosition()
        
    def getPosition(self):
        return p.getBasePositionAndOrientation(self.cylinder_id)[0]
    
    def updatePosition(self):
        self.position = self.getPosition()
        
    def setVelocity(self, x, y):
        p.resetBaseVelocity(self.cylinder_id, [x, y, 0]) 

class PID:
    def __init__(self, error):
        self.max_integral = 10**15
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.previouserror = error
        self.i = 0

    def updateConstants(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
    def getConstants(self):
        return {"kp": self.kp, "ki": self.ki, "kd": self.kd}
    
    def calculateVelocity(self, error):
        period = (1/240.)
        
        # Proportional term
        p = self.kp * error
        
        # Integral term with clamping
        self.i = max(min(self.i + error * period, self.max_integral), -self.max_integral)
        
        # Derivative term
        d = self.kd * (error - self.previouserror) / period
        
        self.previouserror = error
        return p + (self.ki * self.i) + d

if __name__ == "__main__":
    env = PybulletEnvironment()
    try:
        env.run_simulation()
    except KeyboardInterrupt:
        p.disconnect()
        print("Simulation stopped by user.")
