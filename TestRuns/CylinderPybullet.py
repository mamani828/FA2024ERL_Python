import json
import sys, os
import numpy as np
import pybullet as p
import pybullet_data
import time
sys.path.append(os.getcwd()+"/util")
# from util import QtGui
from QtGui import Widget 
from PyQt5.QtWidgets import QApplication

    

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
        Pid_X=PID(0)
        Pid_Y=PID(0)
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
                
                # receiving the position of the object in pybullet
                
                # self.x_vel=Pid_X.calculateVelocity() # add the errors
                # self.y_vel=Pid_Y.calculateVelocity() # add the errors 
                
                
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
    def __init__(self, error):
        self.max_integral=10**15
        self.kp=0
        self.ki=0
        self.kd=0
        self.previouserror= error
        self.i=0
    def updateConstants(self,kp,ki,kd):
        self.kp=kp
        self.ki=ki
        self.kd=kd
        
    def getConstants(self):
        
        return {"kp":self.kp,"ki":self.ki,"kd":self.kd}
    
    def calculateVelocity(self,error):
        
        period=(1/2.40)
        
        p=self.kp*error
        
        i=max(min(i+error*(period),i),-i)
        
        d=self.d*(error-self.previouserror)/(period)
        
        return self.kp*error+self.ki*i+self.kd*d
    
    
    
    # later plan to implement a simple pid to point with the cylinder
    



if __name__ == "__main__":
    env = PybulletEnviorement()
    try:
        env.run_simulation()
    except KeyboardInterrupt:
        p.disconnect()
        print("Simulation stopped by user.")
        
