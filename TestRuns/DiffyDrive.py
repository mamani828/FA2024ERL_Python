import pybullet as p
import pybullet_data
import logging
import time
import numpy as np
# Configure logging
logging.basicConfig(
    filename="simulation.log",
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)


class PybulletEnvironment:
    def __init__(self):
        logging.info("Initializing PyBullet environment...")
        self.physics_client = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        #p.changeDynamics(
        #    lateralFriction=2
        #)
        self.plane_id = p.loadURDF("plane.urdf")

        # choose which to use by setting self.Cylinder to Cylinder() or Robot()
        # self.Cylinder = Cylinder()
        
        self.jackal_robot = Robot()

        # PID controllers
        self.x_pid = PID(kp=1.0, ki=0.01, kd=0.1)
        self.y_pid = PID(kp=1.0, ki=0.01, kd=0.1)

        # Goal position sliders
        self.x_goalpos = p.addUserDebugParameter("X Goal Position", -100, 100, 0)
        self.y_goalpos = p.addUserDebugParameter("Y Goal Position", -100, 100, 0)


        # PID constant sliders
        self.kp_slider = p.addUserDebugParameter("KP", 0, 10, 1)
        self.ki_slider = p.addUserDebugParameter("KI", 0, 1, 0.01)
        self.kd_slider = p.addUserDebugParameter("KD", 0, 1, 0.1)
    
    def getOrientation (self, robot=None):
        pass
    def run_simulation(self, boolean = True):
        logging.info("Starting simulation...")
        print(boolean)
        self.find_joints(self.jackal_robot)
        try:
            while True:
                p.stepSimulation()
                time.sleep(1.0 / 240.0)
                # Update PID constants from sliders

                # Get current position and goal
                current_position = self.Cylinder.getPosition()
                x_goal = p.readUserDebugParameter(self.x_goalpos)
                y_goal = p.readUserDebugParameter(self.y_goalpos)

                # Calculate error
                x_error = x_goal - current_position[0]
                y_error = y_goal - current_position[1]

                # Use PID to calculate velocities
                x_velocity = self.x_pid.calculateVelocity(x_error)
                y_velocity = self.y_pid.calculateVelocity(y_error)
                target_yaw = np.arctan2(y_error, x_error)
                x_yaw = self.Cylinder.getHeading()

                # Update debug text
                # self.update_info_text(x_velocity, y_velocity, current_position)

        except KeyboardInterrupt:
            logging.warning("Simulation stopped by user.")
        except Exception as e:
            logging.error(f"Unexpected error: {e}")
        finally:
            p.disconnect()
            logging.info("Simulation ended.")

    def update_info_text(self, x_velocity, y_velocity, position):
        """
        Update the debug text overlay with the current velocities and position.
        """
        text = f"Velocities: X={x_velocity:.2f}, Y={y_velocity:.2f}\nPosition: {position}"
        p.addUserDebugText(
            text=text,
            textPosition=[0, 0, 3],
            textColorRGB=[1, 1, 1],
            textSize=1.5,
            replaceItemUniqueId=self.info_text_id
        )

        
class Robot:
    def __init__(self):
        self.robot_id = p.loadURDF("./jackal.urdf", basePosition=[0, 0, 0.2])
        
        
        self.position = self.getPosition()
        self.wheels = [1,2,3,4]   # All four wheels

    def getPosition(self):
 
        return p.getBasePositionAndOrientation(self.robot_id)[0]

    def getHeading(self):

        orientation = p.getBasePositionAndOrientation(self.robot_id)[1]
        euler_angles = p.getEulerFromQuaternion(orientation)
        return euler_angles[2]  # Yaw angle
    def getRobotId(self):
        return self.robot_id
    
    
    
    



class PID:
    def __init__(self, kp=0.1, ki=0.01, kd=0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0
        self.dt = 1 / 240.0  # Simulation time step (default is 1/240 seconds)

    def update(self, kp, ki, kd):
        """
        Update the PID constants.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def calculateVelocity(self, error):
        """
        Compute the control output based on the error using PID.
        """
        self.integral += error * self.dt
        derivative = (error - self.previous_error) / self.dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error
        return output

if __name__ == "__main__":
    env = PybulletEnvironment()
    try:
        env.run_simulation()
    except KeyboardInterrupt:
        logging.info("Simulation terminated by user.")
