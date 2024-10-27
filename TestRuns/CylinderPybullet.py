import json
import sys
import numpy as np
import pybullet as p
import pybullet_data
import time
import cv2
from QtGui import Widget
from PyQt5.QtWidgets import QApplication
import sys

    

class PybulletEnviorement():
    def __init__(self):
        super().__init__()
        
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.plane_id = p.loadURDF("plane.urdf")
        self.Cylinder=Cylinder()
        
        
        # initializing the better pyqt gui
        self.app = QApplication(sys.argv)
        self.ControlPanel= Widget()
        self.ControlPanel.show()
        
    
        
    # not being used for now idk if i should delete
    def update_info_text(self):
            p.addUserDebugText(
                text=f"velocities: {self.x_vel, self.y_vel}",
                textPosition=[0, 0, 3],
                textColorRGB=[1, 1, 1],
                textSize=1.5,
                replaceItemUniqueId=self.info_text_id
            )
                
            print(self.Cylinder.position)
    
    # not being used but in future can embed sim within qt
    def getFrame(self):
        width, height, rgbImg, depthImg, segImg = p.getCameraImage(width=640, height=480)
        rgb_array = np.array(rgbImg)
        with open("frame.json", "w") as file:
            json.dump(rgb_array.tolist(), file)  
            
    def run_simulation(self):
        while True:
            p.stepSimulation()
            time.sleep(1./240.)
            self.Cylinder.updatePosition()
            self.ControlPanel.update()
            try:
                # using the new pyqt gui values
                values=self.ControlPanel.get_values()
                self.x_vel=values['X_velocity']
                self.y_vel=values['Y_velocity']
                self.x_goalpos=values['X_goalpos']
                self.y_goalpos=values['Y_goalpos']
                self.Cylinder.setVelocity(self.x_vel,self.y_vel)
                
                # print(self.ControlPanel.get_values())
            except:
                continue
            
    
         
class Cylinder: 
    def __init__(self):
        cylinder_height=2
        self.cylinder_id=p.loadURDF("cylinder.urdf",basePosition=[0, 0, cylinder_height / 2])
        self.position=self.getPosition()
    def getPosition(self):
        return p.getBasePositionAndOrientation(self.cylinder_id)[0]
    
    def updatePosition(self):
        self.position=self.getPosition()
        
    def setVelocity(self,x,y):
        p.resetBaseVelocity(self.cylinder_id, [x, y, 0]) # ignoring setting heading since its a cylinder so heading doesnt matter
    
    
class PID:
    def __init__(self,time):
        self.kp=0
        self.ki=0
        self.kd=0
        self.previoustime=time
    def update(self,kp,ki,kd):
        self.kp=kp
        self.ki=ki
        self.kd=kd
        
    def getConstants(self):
        
        return {"kp":self.kp,"ki":self.ki,"kd":self.kd}
    
    def calculateVelocity(self,error):
        time=(1/2.40)
        return self.kp*error
    
    
    
    # later plan to implement a simple pid to point with the cylinder
    



if __name__ == "__main__":
    env = PybulletEnviorement()
    try:
        env.run_simulation()
    except KeyboardInterrupt:
        p.disconnect()
        print("Simulation stopped by user.")
        
