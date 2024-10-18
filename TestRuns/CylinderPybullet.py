import pybullet as p
import pybullet_data
import time

class PybulletEnviorement:
    def __init__(self):
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.plane_id = p.loadURDF("plane.urdf")
        self.Cylinder=Cylinder()
    
    def run_simulation(self):
        while True:
            p.stepSimulation()
            time.sleep(1./240.)
            self.Cylinder.updatePosition()
            self.Cylinder.setVelocity(1,1)
            
    
        
         
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
    
    
        



if __name__ == "__main__":
    env = PybulletEnviorement()
    try:
        env.run_simulation()
    except KeyboardInterrupt:
        p.disconnect()
        print("Simulation stopped by user.")
        
        
