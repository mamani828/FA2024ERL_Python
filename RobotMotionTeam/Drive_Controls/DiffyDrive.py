# import pybullet as p
# import pybullet_data
# import logging
# import time
# import numpy as np
# # Configure logging
# logging.basicConfig(
#     filename="./RobotMotionTeam/Simulation_logging/Diffy_simulation.log",
#     level=logging.INFO,
#     format="%(asctime)s - %(levelname)s - %(message)s"
# )


# class PybulletEnvironment:
#     def __init__(self):
#         logging.info("Initializing PyBullet environment...")
#         self.physics_client = p.connect(p.GUI)
#         p.setAdditionalSearchPath(pybullet_data.getDataPath())
#         p.setGravity(0, 0, -9.81)
#         self.plane_id = p.loadURDF("plane.urdf")

#         # choose which to use by setting self.Cylinder to Cylinder() or Robot()
#         # self.Cylinder = Cylinder()
        
#         self.jackal_robot = Robot()

#         # Velocity position sliders
#         self.v_forward_slider = p.addUserDebugParameter("Linear Velocity", -50, 50, 0)
#         self.v_angular_slider = p.addUserDebugParameter("Angular Velocity", -200, 200, 0)

    
#     def run_simulation(self, boolean = True):
#         logging.info("Starting simulation...")
#         print(boolean)
#         try:
#             while True:
#                 p.stepSimulation()
#                 time.sleep(1.0 / 240.0)
#                 self.v_forward = p.readUserDebugParameter(self.v_forward_slider)
#                 self.v_angular = p.readUserDebugParameter(self.v_angular_slider)
                
#                 self.jackal_robot.inverse_kinematics(self.v_forward,self.v_angular)
#                 self.jackal_robot.setVelocity()
                

#         except KeyboardInterrupt:
#             logging.warning("Simulation stopped by user.")
#         except Exception as e:
#             logging.error(f"Unexpected error: {e}")
#         finally:
#             p.disconnect()
#             logging.info("Simulation ended.")

#     def update_info_text(self, x_velocity, y_velocity, position):
#         """
#         Update the debug text overlay with the current velocities and position.
#         """
#         text = f"Velocities: X={x_velocity:.2f}, Y={y_velocity:.2f}\nPosition: {position}"
#         p.addUserDebugText(
#             text=text,
#             textPosition=[0, 0, 3],
#             textColorRGB=[1, 1, 1],
#             textSize=1.5,
#             replaceItemUniqueId=self.info_text_id
#         )

        
# class Robot:
#     def __init__(self):
#         self.robot_id = p.loadURDF("./RobotMotionTeam/urdf/jackal.urdf", basePosition=[0, 0, 0.2])
        
        
#         self.position = self.getPosition()
#         self.wheels = [1,2,3,4]   # All four wheels
#         self.wheels_left=[1,3]
#         self.wheels_right=[2,4]
#         self.track_radius=0.187795 
#         self.vl=0
#         self.vr=0
#         """
#         Wheel Index 1: Front Left
#         Wheel Index 2: Front Right
#         Wheel Index 3: Back Left
#         Wheel Index 4: Back Right
#         """

#     def getPosition(self):
 
#         return p.getBasePositionAndOrientation(self.robot_id)[0]

#     def getHeading(self):

#         orientation = p.getBasePositionAndOrientation(self.robot_id)[1]
#         euler_angles = p.getEulerFromQuaternion(orientation)
#         return euler_angles[2]  # Yaw angle
#     def getRobotId(self):
#         return self.robot_id
#     def setVelocity(self):

#             for wheel in self.wheels_left:
#                 p.setJointMotorControl2(
#                     bodyIndex=self.robot_id,
#                     jointIndex=wheel,
#                     controlMode=p.VELOCITY_CONTROL,
#                     targetVelocity=self.vl,
#                     force = 32 # i think the torque on a jackal
#                 )
#             for wheel in self.wheels_right:
#                 p.setJointMotorControl2(
#                     bodyIndex=self.robot_id,
#                     jointIndex=wheel,
#                     controlMode=p.VELOCITY_CONTROL,
#                     targetVelocity=self.vr,
#                     force = 32 # i think the torque on a jackal
#                 )
#     def inverse_kinematics(self,v_f, v0): # converting forward+angular velocity into wheel velocities
#         self.vr=v_f+self.track_radius*v0        # velocity of left wheels
#         self.vl=v_f-self.track_radius*v0       # velocity of right wheels 
        
        
#     """ wheel velocity: [wheel front left, wheel front right, wheel back left, wheel back right]"""
#     # def forward_kinematics(self,wheel_vels): # converting wheel velocities into forward+angular velocity 
        
    
    


# if __name__ == "__main__":
#     env = PybulletEnvironment()
#     try:
#         env.run_simulation()
#     except KeyboardInterrupt:
#         logging.info("Simulation terminated by user.")


from math import comb 


# # Configure logging
# logging.basicConfig(
#     filename="./RobotMotionTeam/Simulation_logging/Diffy_simulation.log",
#     level=logging.INFO,
#     format="%(asctime)s - %(levelname)s - %(message)s"
# )

# class PybulletEnvironment:
#     def __init__(self):
#         logging.info("Initializing PyBullet environment...")
#         self.physics_client = p.connect(p.GUI)
#         p.setAdditionalSearchPath(pybullet_data.getDataPath())
#         p.setGravity(0, 0, -9.81)
#         self.plane_id = p.loadURDF("plane.urdf")

#         self.jackal_robot = Robot()

#         # Velocity sliders
#         self.v_forward_slider = p.addUserDebugParameter("Linear Velocity", -5, 5, 0)
#         self.v_angular_slider = p.addUserDebugParameter("Angular Velocity", -5, 5, 0)

#         # Bezier curve control points
#         self.control_points = [
#             p.addUserDebugParameter(f"Control Point {i} X", -10, 10, 0)
#             for i in range(4)
#         ] + [
#             p.addUserDebugParameter(f"Control Point {i} Y", -10, 10, 0)
#             for i in range(4)
#         ]

#     def get_control_points(self):
#         # Retrieve control points from sliders
#         x_coords = [p.readUserDebugParameter(self.control_points[i]) for i in range(4)]
#         y_coords = [p.readUserDebugParameter(self.control_points[i + 4]) for i in range(4)]
#         return list(zip(x_coords, y_coords))

#     def bezier_curve(self, control_points, num_points=100):
#         # Generate a Bezier curve from control points
#         def bezier_point(t, points):
#             n = len(points) - 1
#             return sum(
#                 comb(n, i) * (1 - t) ** (n - i) * t**i * np.array(points[i])
#                 for i in range(n + 1)
#             )

#         return [bezier_point(t, control_points) for t in np.linspace(0, 1, num_points)]

#     def run_simulation(self):
#         logging.info("Starting simulation...")
#         try:
#             while True:
#                 p.stepSimulation()
#                 time.sleep(1.0 / 240.0)

#                 # Get control points and compute the Bezier path
#                 control_points = self.get_control_points()
#                 bezier_path = self.bezier_curve(control_points)

#                 # Follow the Bezier path
#                 for point in bezier_path:
#                     robot_pos = self.jackal_robot.getPosition()
#                     angle_to_target = np.arctan2(
#                         point[1] - robot_pos[1], point[0] - robot_pos[0]
#                     )
#                     heading = self.jackal_robot.getHeading()

#                     # Compute angular velocity to align with the target
#                     angular_velocity = angle_to_target - heading
#                     linear_velocity = 2.0  # Constant forward speed

#                     self.jackal_robot.inverse_kinematics(linear_velocity, angular_velocity)
#                     self.jackal_robot.setVelocity()

#         except KeyboardInterrupt:
#             logging.warning("Simulation stopped by user.")
#         except Exception as e:
#             logging.error(f"Unexpected error: {e}")
#         finally:
#             p.disconnect()
#             logging.info("Simulation ended.")

# class Robot:
#     def __init__(self):
#         self.robot_id = p.loadURDF(
#             "./RobotMotionTeam/urdf/jackal.urdf", basePosition=[0, 0, 0.2]
#         )

#         self.position = self.getPosition()
#         self.wheels_left = [1, 3]  # Left wheels
#         self.wheels_right = [2, 4]  # Right wheels
#         self.track_radius = 0.187795  # Half of the distance between wheels
#         self.vl = 0  # Velocity for left wheels
#         self.vr = 0  # Velocity for right wheels

#     def getPosition(self):
#         return p.getBasePositionAndOrientation(self.robot_id)[0]

#     def getHeading(self):
#         orientation = p.getBasePositionAndOrientation(self.robot_id)[1]
#         euler_angles = p.getEulerFromQuaternion(orientation)
#         return euler_angles[2]  # Yaw angle

#     def setVelocity(self):
#         for wheel in self.wheels_left:
#             p.setJointMotorControl2(
#                 bodyIndex=self.robot_id,
#                 jointIndex=wheel,
#                 controlMode=p.VELOCITY_CONTROL,
#                 targetVelocity=self.vl,
#                 force=32
#             )
#         for wheel in self.wheels_right:
#             p.setJointMotorControl2(
#                 bodyIndex=self.robot_id,
#                 jointIndex=wheel,
#                 controlMode=p.VELOCITY_CONTROL,
#                 targetVelocity=self.vr,
#                 force=32
#             )

#     def inverse_kinematics(self, v_f, v0):
#         self.vr = v_f + self.track_radius * v0
#         self.vl = v_f - self.track_radius * v0

import pybullet as p
import pybullet_data
import logging
import time
import numpy as np
import math
# from RobotMotionTeam.Utils.Controllers.Pid import PID
# from RobotMotionTeam.Utils.Controllers.PurePursuit import PurePursuit
# Configure logging
# from ..Utils.Controllers.Pid import Pid

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

        self.jackal_robot = Robot()

        # Velocity position sliders
        self.v_forward_slider = p.addUserDebugParameter("Linear Velocity", -50, 50, 0)
        self.v_angular_slider = p.addUserDebugParameter("Angular Velocity", -100, 100, 0)
        
        self.goalPointX=p.addUserDebugParameter("Goal Point X", -50, 50, 0)
        self.goalPointY=p.addUserDebugParameter("Goal Point Y", -50, 50, 0)
        
        self.kp=p.addUserDebugParameter("kp", -10, 10, 4)
        self.ki=p.addUserDebugParameter("ki", -10, 10, 0)
        self.kd=p.addUserDebugParameter("kd", -10, 10, 0.1)

    
    def run_simulation(self, boolean = True):
        # Velocity sliders
        self.v_forward_slider = p.addUserDebugParameter("Linear Velocity", 0, 10, 2)

        # Initialize control points for Bezier curve
        self.control_points = [
            np.array([0, 0, 0.2]),
            np.array([2, 1, 0.2]),
            np.array([4, -1, 0.2]),
            np.array([6, 0, 0.2])
        ]

        # Place spheres at control points
        self.sphere_ids = [
            p.createMultiBody(baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_SPHERE, radius=0.1),
                              baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=0.1, rgbaColor=[1, 0, 0, 1]),
                              basePosition=cp.tolist())
            for cp in self.control_points
        ]

        # Add debug sliders for each control point (X, Y, Z)
        self.sliders = []
        for i, cp in enumerate(self.control_points):
            self.sliders.append(
                p.addUserDebugParameter(f"Control Point {i+1} X", -5, 5, cp[0])
            )
            self.sliders.append(
                p.addUserDebugParameter(f"Control Point {i+1} Y", -5, 5, cp[1])
            )
            self.sliders.append(
                p.addUserDebugParameter(f"Control Point {i+1} Z", -5, 5, cp[2])
            )

        # Draw the initial path
        self.draw_bezier_path()

        # Path following parameter (0 -> start, 1 -> end)
        self.t = 0.0

    def draw_bezier_path(self):
        """Draws the Bezier curve based on the current control points."""
        p.removeAllUserDebugItems()  # Clear previous debug lines
        for i in range(len(self.control_points) - 1):
            p.addUserDebugLine(
                self.control_points[i].tolist(),
                self.control_points[i + 1].tolist(),
                lineColorRGB=[0, 1, 0],
                lineWidth=2
            )

    def update_control_points(self):
        """Updates the control points based on the slider values."""
        for i, slider in enumerate(self.sliders):
            control_point_idx = i // 3  # Each control point has 3 sliders (X, Y, Z)
            axis_idx = i % 3  # 0 for X, 1 for Y, 2 for Z
            value = p.readUserDebugParameter(slider)
            self.control_points[control_point_idx][axis_idx] = value

    def get_bezier_curve(self, t):
        """Computes a point on the Bezier curve at parameter t."""
        cp = self.control_points
        return (1 - t)**3 * cp[0] + 3 * (1 - t)**2 * t * cp[1] + 3 * (1 - t) * t**2 * cp[2] + t**3 * cp[3]

    def run_simulation(self):
        logging.info("Starting simulation...")
        

            
        self.linear_pid=PID(kp=1.0, ki=0, kd=0.1)
        while True:
                p.stepSimulation()
                time.sleep(1.0 / 240.0)
                self.v_forward = p.readUserDebugParameter(self.v_forward_slider)
                self.v_angular = p.readUserDebugParameter(self.v_angular_slider)
                
                self.jackal_robot.inverse_kinematics(self.v_forward,self.v_angular)
                self.jackal_robot.setVelocity()
                

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
        self.robot_id = p.loadURDF("./RobotMotionTeam/urdf/jackal.urdf", basePosition=[0, 0, 0.2])
        
        
        self.position = self.getPosition()
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
    def getRobotId(self):
        return self.robot_id
    def setVelocity(self):

            for wheel in self.wheels_left:
                p.setJointMotorControl2(
                    bodyIndex=self.robot_id,
                    jointIndex=wheel,
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=self.vl,
                    force = 32 # i think the torque on a jackal
                )
            for wheel in self.wheels_right:
                p.setJointMotorControl2(
                    bodyIndex=self.robot_id,
                    jointIndex=wheel,
                    controlMode=p.VELOCITY_CONTROL,
                    targetVelocity=self.vr,
                    force = 32 # i think the torque on a jackal
                )
    def inverse_kinematics(self,v_f, v0): # converting forward+angular velocity into wheel velocities
        self.vr=v_f+self.track_radius*v0        # velocity of left wheels
        self.vl=v_f-self.track_radius*v0       # velocity of right wheels 
        
        
    """ wheel velocity: [wheel front left, wheel front right, wheel back left, wheel back right]"""
    # def forward_kinematics(self,wheel_vels): # converting wheel velocities into forward+angular velocity 
        
    
    


if __name__ == "__main__":
    env = PybulletEnvironment()
    try:
        env.run_simulation()
    except KeyboardInterrupt:
        logging.info("Simulation terminated by user.")
