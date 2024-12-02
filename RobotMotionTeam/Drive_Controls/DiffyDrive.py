import pybullet as p
import pybullet_data
import logging
import time
import numpy as np
import math
from RobotMotionTeam import PID
from RobotMotionTeam import Bezier
from RobotMotionTeam import PurePursuit
# from RobotMotionTeam.Utils.Controllers.Pid import PID
# from RobotMotionTeam.Utils.Controllers.PurePursuit import PurePursuit
# Configure logging
# from ..Utils.Controllers.Pid import Pid
# import ..Utils.Controllers.Pid 

logging.basicConfig(
    filename="./RobotMotionTeam/Simulation_logging/Diffy_simulation.log",
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)


class PybulletEnvironment:
    def __init__(self):
        logging.info("Initializing PyBullet environment...")
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.plane_id = p.loadURDF("plane.urdf")
        p.changeDynamics(self.plane_id, -1, lateralFriction=1.0)
        # choose which to use by setting self.Cylinder to Cylinder() or Robot()
        # self.Cylinder = Cylinder()
        
        self.jackal_robot = Robot()


        self.sliders=[]
        self.path=Bezier(p,self.sliders)    
        self.create_sliders()
    def create_sliders(self):
        
        # self.goalPointX=p.addUserDebugParameter("Goal Point X", -50, 50, 0)
        # self.goalPointY=p.addUserDebugParameter("Goal Point Y", -50, 50, 0)
        
        self.kp_linear=p.addUserDebugParameter("kp-linear", 0, 100, 40) 
        self.ki_linear=p.addUserDebugParameter("ki-linear", 0, 10, 0)
        self.kd_linear=p.addUserDebugParameter("kd-linear", 0, 10, 1.0)
        
        self.kp_angular=p.addUserDebugParameter("kp-angular", 0, 100, 50) 
        self.ki_angular=p.addUserDebugParameter("ki-angular", 0, 50, 0)
        self.kd_angular=p.addUserDebugParameter("kd-angular", 0, 50, 0.5)
            
            
            
    def run_simulation(self, boolean = True):
        logging.info("Starting simulation...")
        
        # initializing pid and its corresponding plot
        self.linear_pid=PID(kp=1.0, ki=0, kd=0.01)
        self.angular_pid=PID(kp=1.0, ki=0, kd=0.01)
        self.path.get_Path()
        self.path.draw_bezier_path()
        self.follower=PurePursuit(self.jackal_robot,self.path.get_Path(1000),0.8)
        
        while True:
                p.stepSimulation()
                time.sleep(1.0 / 240.0)
                self.read_sliders()
                
                
                # localization
                self.jackal_robot.localize()
                self.current_x=self.jackal_robot.position[0]
                self.current_y=self.jackal_robot.position[1]
                self.current_h=self.jackal_robot.position[2]

                # finding goal point
                goal=self.follower.find_lookahead_point()
                self.x_goal=goal[0]
                self.y_goal=goal[1]
                print(self.x_goal,self.y_goal)
                # showing the lookahead intersection point
                p.addUserDebugLine([self.x_goal,self.y_goal-0.1,0.2],
                                    [self.x_goal,self.y_goal+0.1 , 0.2],
                                    lineColorRGB=[1, 0, 0], 
                                    lineWidth=300)
            
                self.x_error=self.x_goal-self.current_x
                self.y_error=self.y_goal-self.current_y
                self.h_error=self.angle_wrap(math.atan2(self.y_error,self.x_error)-self.current_h)
                
                # calculate with pid
                self.linear_velocity=self.linear_pid.calculateVelocity(self.x_error)
                self.angular_velocity=self.angular_pid.calculateVelocity(self.h_error)
                
                print(self.x_error)
                # print(self.linear_velocity,self.angular_velocity)
                # setting the velocities
                self.jackal_robot.inverse_kinematics(self.linear_velocity,self.angular_velocity)
                self.jackal_robot.setVelocity()
                
                
                
                
                
                # print(self.jackal_robot.position)
              
                #logging data
                logging.info(f"Position: {', '.join(str(i) for i in self.jackal_robot.position)}, Errors: X={self.x_error}, Y={self.y_error}, "
                             f"Velocities: X={self.linear_velocity}, Heading={self.h_error}")

                

    def angle_wrap(self,radians):
        while radians > math.pi:
            radians -= 2 * math.pi
        while radians < -math.pi:
            radians += 2 * math.pi
        return radians
    def read_sliders(self):
        try:
        #     self.x_goal=p.readUserDebugParameter(self.goalPointX)
        #     self.y_goal=p.readUserDebugParameter(self.goalPointY)
            self.linear_pid.update(float(p.readUserDebugParameter(self.kp_linear)),float(p.readUserDebugParameter(self.ki_linear)),float(p.readUserDebugParameter(self.kd_linear)))
            self.angular_pid.update(float(p.readUserDebugParameter(self.kp_angular)),float(p.readUserDebugParameter(self.ki_angular)),float(p.readUserDebugParameter(self.kd_angular)))
        except:
            print("Read failed")
            return
        
        
        
class Robot:
    def __init__(self):
        self.robot_id = p.loadURDF("./RobotMotionTeam/urdf/jackal.urdf", basePosition=[0, 0, 0.2])
        
        self.position=self.localize()
        self.wheels = [1,2,3,4]   # All four wheels
        self.wheels_left=[1,3]
        self.wheels_right=[2,4]
        self.track_radius=0.187795 
        self.vl=0
        self.vr=0
        
        
        
        """
        Wheel Index 1: Front Left
        Wheel Index 2: Front Right
        Wheel Index 3: Back Left
        Wheel Index 4: Back Right
        """

    def getPosition(self):
        return p.getBasePositionAndOrientation(self.robot_id)[0]

    def getHeading(self):
        orientation = p.getBasePositionAndOrientation(self.robot_id)[1]
        euler_angles = p.getEulerFromQuaternion(orientation)
        return euler_angles[2]  # Yaw angle
    
    def localize(self): # outputs and sets position in x, y, heading format
        self.position=[]# clearing old position
        self.position.append(self.getPosition()[0])
        self.position.append(self.getPosition()[1])
        self.position.append(self.getHeading())
        return self.position
        
    def getRobotId(self):
        return self.robot_id
    def setVelocity(self):

            for wheel in self.wheels_left:
                p.setJointMotorControl2(
                    bodyIndex=self.robot_id,
                    jointIndex=wheel,
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=self.vl,
                    force = 50 # i think the torque on a jackal
                )
            for wheel in self.wheels_right:
                p.setJointMotorControl2(
                    bodyIndex=self.robot_id,
                    jointIndex=wheel,
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=self.vr,
                    force = 50 # i think the torque on a jackal
                )
    def inverse_kinematics(self,v_f, v0): # converting forward+angular velocity into wheel velocities
        self.vr=v_f+self.track_radius*v0        # velocity of left wheels
        self.vl=v_f-self.track_radius*v0       # velocity of right wheels 
        
        
        


if __name__ == "__main__":
    env = PybulletEnvironment()
    try:
        env.run_simulation()
    except KeyboardInterrupt:
        logging.info("Simulation terminated by user.")
    
