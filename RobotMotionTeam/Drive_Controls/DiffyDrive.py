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

# Configure logging
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
        try:
            while True:
                p.stepSimulation()
                time.sleep(1.0 / 480.0)

                # Update control points based on slider input
                self.update_control_points()

                # Redraw the path based on updated control points
                self.draw_bezier_path()

                # Read velocity slider
                v_forward = p.readUserDebugParameter(self.v_forward_slider)

                # Update path following parameter t (increment to move along the path)
                self.t += 0.005  # You can adjust the increment for smoother or faster motion
                if self.t > 1:
                    self.t = 0  # Loop back to the start of the path

                # Get the next target point on the Bezier curve
                target_position = self.get_bezier_curve(self.t)

                # Snap the robot to the closest point on the path if moved manually
                self.jackal_robot.snap_to_path(target_position)

                # Move the robot towards the target point
                self.jackal_robot.move_towards(target_position, v_forward)

        except KeyboardInterrupt:
            logging.warning("Simulation stopped by user.")
        except Exception as e:
            logging.error(f"Unexpected error: {e}")
        finally:
            p.disconnect()
            logging.info("Simulation ended.")

class Robot:
    def __init__(self):
        self.robot_id = p.loadURDF("./RobotMotionTeam/urdf/jackal.urdf", basePosition=[0, 0, 0.2])
        self.position = self.getPosition()
        self.track_radius = 0.187795
        self.vl = 0
        self.vr = 0

    def getPosition(self):
        return p.getBasePositionAndOrientation(self.robot_id)[0]

    def setVelocity(self):
        for wheel in [1, 3]:  # Left wheels
            p.setJointMotorControl2(self.robot_id, wheel, p.VELOCITY_CONTROL, self.vl, force=32)
        for wheel in [2, 4]:  # Right wheels
            p.setJointMotorControl2(self.robot_id, wheel, p.VELOCITY_CONTROL, self.vr, force=32)

    def move_towards(self, target_position, v_forward):
        """Move the robot towards the target position."""
        current_position = np.array(self.getPosition())
        direction = target_position - current_position
        distance = np.linalg.norm(direction)

        # Normalize the direction vector
        direction = direction / distance if distance != 0 else direction

        # Compute heading (angle to target)
        angle_to_target = np.arctan2(direction[1], direction[0])
        current_orientation = p.getBasePositionAndOrientation(self.robot_id)[1]
        current_euler = p.getEulerFromQuaternion(current_orientation)
        heading = angle_to_target - current_euler[2]

        # Set velocity for path following
        self.vr = v_forward + self.track_radius * heading
        self.vl = v_forward - self.track_radius * heading

        self.setVelocity()

    def snap_to_path(self, target_position):
        """Snap the robot to the closest point on the path."""
        current_position = np.array(self.getPosition())
        direction_to_target = target_position - current_position
        distance_to_target = np.linalg.norm(direction_to_target)

        # Only snap if the robot is too far from the path
        if distance_to_target > 0.2:  # Threshold for snapping
            p.resetBasePositionAndOrientation(self.robot_id, target_position.tolist(), [0, 0, 0, 1])

if __name__ == "__main__":
    env = PybulletEnvironment()
    env.run_simulation()

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

#         self.jackal_robot = Robot()

#         # Velocity sliders
#         self.v_forward_slider = p.addUserDebugParameter("Linear Velocity", 0, 10, 2)

#         # Initialize control points for Bezier curve
#         self.control_points = [
#             np.array([0, 0, 0.2]),
#             np.array([2, 1, 0.2]),
#             np.array([4, -1, 0.2]),
#             np.array([6, 0, 0.2])
#         ]

#         # Place spheres at control points
#         self.sphere_ids = [
#             p.createMultiBody(baseCollisionShapeIndex=p.createCollisionShape(p.GEOM_SPHERE, radius=0.1),
#                               baseVisualShapeIndex=p.createVisualShape(p.GEOM_SPHERE, radius=0.1, rgbaColor=[1, 0, 0, 1]),
#                               basePosition=cp.tolist())
#             for cp in self.control_points
#         ]

#         # Add debug sliders for each control point (X, Y, Z)
#         self.sliders = []
#         for i, cp in enumerate(self.control_points):
#             self.sliders.append(
#                 p.addUserDebugParameter(f"Control Point {i+1} X", -5, 5, cp[0])
#             )
#             self.sliders.append(
#                 p.addUserDebugParameter(f"Control Point {i+1} Y", -5, 5, cp[1])
#             )
#             self.sliders.append(
#                 p.addUserDebugParameter(f"Control Point {i+1} Z", -5, 5, cp[2])
#             )

#         # Variables to store line IDs for Bezier curve and control points
#         self.curve_line_ids = []
#         self.control_line_ids = []

#         # Draw the initial path
#         self.draw_bezier_path()

#     def draw_bezier_path(self, resolution=100):
#         """Draws the Bezier curve and the control point lines, both of which will persist and update."""
#         # Remove old lines if they exist
#         for line_id in self.curve_line_ids:
#             p.removeUserDebugItem(line_id)
#         for line_id in self.control_line_ids:
#             p.removeUserDebugItem(line_id)

#         # Redraw the Bezier curve (smooth path)
#         curve_points = []
#         for t in np.linspace(0, 1, resolution):  # Generate points from t=0 to t=1
#             point = self.get_bezier_curve(t)
#             curve_points.append(point)

#         # Store the new Bezier curve lines
#         self.curve_line_ids = []
#         for i in range(len(curve_points) - 1):
#             line_id = p.addUserDebugLine(
#                 curve_points[i].tolist(),
#                 curve_points[i + 1].tolist(),
#                 lineColorRGB=[0, 1, 0],  # Green color for the curve
#                 lineWidth=2
#             )
#             self.curve_line_ids.append(line_id)

#         # Store the new control points lines
#         self.control_line_ids = []
#         for i in range(len(self.control_points) - 1):
#             line_id = p.addUserDebugLine(
#                 self.control_points[i].tolist(),
#                 self.control_points[i + 1].tolist(),
#                 lineColorRGB=[1, 0, 0],  # Red color for control points lines
#                 lineWidth=2
#             )
#             self.control_line_ids.append(line_id)

#     def update_control_points(self):
#         """Updates the control points based on the slider values."""
#         for i, slider in enumerate(self.sliders):
#             control_point_idx = i // 3  # Each control point has 3 sliders (X, Y, Z)
#             axis_idx = i % 3  # 0 for X, 1 for Y, 2 for Z
#             value = p.readUserDebugParameter(slider)
#             self.control_points[control_point_idx][axis_idx] = value

#     def get_bezier_curve(self, t):
#         """Computes a point on the Bezier curve at parameter t."""
#         cp = self.control_points
#         return (1 - t)**3 * cp[0] + 3 * (1 - t)**2 * t * cp[1] + 3 * (1 - t) * t**2 * cp[2] + t**3 * cp[3]

#     def run_simulation(self):
#         logging.info("Starting simulation...")
#         try:
#             while True:
#                 p.stepSimulation()
#                 time.sleep(1.0 / 480.0)

#                 # Update control points based on slider input
#                 self.update_control_points()

#                 # Redraw the path based on updated control points
#                 self.draw_bezier_path()

#                 # Read velocity slider
#                 v_forward = p.readUserDebugParameter(self.v_forward_slider)

#                 # Update path following parameter t (increment to move along the path)
#                 self.t += 0.005  # You can adjust the increment for smoother or faster motion
#                 if self.t > 1:
#                     self.t = 0  # Loop back to the start of the path

#                 # Get the next target point on the Bezier curve
#                 target_position = self.get_bezier_curve(self.t)

#                 # Snap the robot to the closest point on the path if moved manually
#                 self.jackal_robot.snap_to_path(target_position)

#                 # Move the robot towards the target point
#                 self.jackal_robot.move_towards(target_position, v_forward)

#         except KeyboardInterrupt:
#             logging.warning("Simulation stopped by user.")
#         except Exception as e:
#             logging.error(f"Unexpected error: {e}")
#         finally:
#             p.disconnect()
#             logging.info("Simulation ended.")


# class Robot:
#     def __init__(self):
#         self.robot_id = p.loadURDF("./RobotMotionTeam/urdf/jackal.urdf", basePosition=[0, 0, 0.2])
#         self.position = self.getPosition()
#         self.track_radius = 0.187795
#         self.vl = 0
#         self.vr = 0

#     def getPosition(self):
#         return p.getBasePositionAndOrientation(self.robot_id)[0]

#     def setVelocity(self):
#         for wheel in [1, 3]:  # Left wheels
#             p.setJointMotorControl2(self.robot_id, wheel, p.VELOCITY_CONTROL, self.vl, force=32)
#         for wheel in [2, 4]:  # Right wheels
#             p.setJointMotorControl2(self.robot_id, wheel, p.VELOCITY_CONTROL, self.vr, force=32)

#     def move_towards(self, target_position, v_forward):
#         """Move the robot towards the target position."""
#         current_position = np.array(self.getPosition())
#         direction = target_position - current_position
#         distance = np.linalg.norm(direction)

#         # Normalize the direction vector
#         direction = direction / distance if distance != 0 else direction

#         # Compute heading (angle to target)
#         angle_to_target = np.arctan2(direction[1], direction[0])
#         current_orientation = p.getBasePositionAndOrientation(self.robot_id)[1]
#         current_euler = p.getEulerFromQuaternion(current_orientation)
#         heading = angle_to_target - current_euler[2]

#         # Set velocity for path following
#         self.vr = v_forward + self.track_radius * heading
#         self.vl = v_forward - self.track_radius * heading

#         self.setVelocity()

#     def snap_to_path(self, target_position):
#         """Snap the robot to the closest point on the path."""
#         current_position = np.array(self.getPosition())
#         direction_to_target = target_position - current_position
#         distance_to_target = np.linalg.norm(direction_to_target)

#         # Only snap if the robot is too far from the path
#         if distance_to_target > 0.2:  # Threshold for snapping
#             p.resetBasePositionAndOrientation(self.robot_id, target_position.tolist(), [0, 0, 0, 1])

# if __name__ == "__main__":
#     env = PybulletEnvironment()
#     env.run_simulation()
