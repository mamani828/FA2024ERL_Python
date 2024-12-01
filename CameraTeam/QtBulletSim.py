import sys
import os
import yaml
import threading

import pybullet as p
import pybullet_data
import numpy as np

from PyQt6.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel, QSlider
from PyQt6.QtCore import QTimer, Qt, QThread
from PyQt6.QtGui import QPixmap, QImage
from PyQt6 import QtWidgets

from utils.Sensors import Camera, Lidar
from utils.QtMap import RobotMap
from utils.SimulationObjects import Robot, Object


SENSOR_CONFIG_PATH = os.path.join(os.path.dirname(__file__),
                                  'config/sensors.yaml')
DEFAULT_GRID_SIZE = 100
SIM_TIME_CONSTANT = 4  # How often the simulation is updated
SYNTH_CAMERA_UPDATE_INTERVAL = 30
DEBUG_CAMERA_UPDATE_INTERVAL = 10
MAX_RAY_NUMBER = 75
MAX_START_ANGLE = 360
MAX_END_ANGLE = 360
MAX_RAY_LENGTH = 10
MAX_ROBOT_VELOCITY = 100
MAX_FORCE = 2
WALL_MASS = 0  # Mass of 0 to make the wall static
WALL_COLOR = [0.7, 0.7, 0.7, 1]  # Gray color for the wall


class CubeCreator:
    def __init__(self):
        self.cube_count = 0
        self.button_id = p.addUserDebugParameter("Create Cube", 1, 0, 1)
        self.prev_button_state = p.readUserDebugParameter(self.button_id)
        self.create_info_text()

    def create_info_text(self):
        self.info_text_id = p.addUserDebugText(
            text="Cubes created: 0",
            textPosition=[0, 0, 3],
            textColorRGB=[1, 1, 1],
            textSize=1.5
        )

    def update_info_text(self, x, y):
            p.addUserDebugText(
                text=f"Cubes created: {self.cube_count}",
                textPosition=[x, y, 3],
                textColorRGB=[1, 1, 1],
                textSize=1.5,
                replaceItemUniqueId=self.info_text_id
            )

    def update_cube_button(self, robot_pos):
        current_button_state = p.readUserDebugParameter(self.button_id)
        if current_button_state != self.prev_button_state:
            self.create_new_cube()
            self.cube_count += 1
            self.update_info_text(robot_pos[0], robot_pos[1])
            self.prev_button_state = current_button_state

    @staticmethod
    def create_new_cube():
        cube_size = 0.3
        x = np.random.normal(0, 4)
        y = np.random.normal(0, 4)
        position = [x, y, cube_size/2]
        orientation = p.getQuaternionFromEuler([0, 0, 0])
        color = list(np.random.uniform(0, 1, 3)) + [1]
        visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX,
                                        halfExtents=[cube_size/2]*3,
                                        rgbaColor=color)

        collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX,
                                                halfExtents=[cube_size/2]*3)

        p.createMultiBody(baseMass=1, baseVisualShapeIndex=visual_shape,
                        baseCollisionShapeIndex=collision_shape,
                        basePosition=position,
                        baseOrientation=orientation)

class SimulationApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.init_simulation()
        #self.slider_ui()
        #self.initMap()
        self.simulation_running = True

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_simulation)
        self.timer.start(SIM_TIME_CONSTANT)

    def init_simulation(self):
        self.physicsClient = p.connect(p.GUI)
        p.setGravity(0, 0, -9.81)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        coordinates = [0, 0, 0]  # initial coordinates and orientation for the plane
        orientation = [0, 0, 0]  # Euler Angles Roll, Pitch, Yaw
        initial_orientation = p.getQuaternionFromEuler(orientation)  # Converting to Quaternion
        self.plane_id = p.loadURDF("plane.urdf", coordinates, initial_orientation)

        # First, create an instance of the Object class
        object_instance = Object(coordinates=[5, 0, 1], color=[1, 0, 0, 1])  # Example object (coordinates, color)

        # Now, call the create_wall method on that instance
        WALL_SIZE = [0.2, 5, 1]  # Wall dimensions (length, width, height)
        wall1 = object_instance.create_wall(position=[2, 0, 0], WALL_SIZE=WALL_SIZE, WALL_COLOR=WALL_COLOR)
        wall2 = object_instance.create_wall(position=[-2, 0, 0], WALL_SIZE=WALL_SIZE, WALL_COLOR=WALL_COLOR)

        camera_config = SimulationApp.open_yaml("camera_configs")
        lidar_config = SimulationApp.open_yaml("lidar_configs")
        robot_config = SimulationApp.open_yaml("robot_configs")

        self.robot = Robot(robot_config)
        self.robot_id = self.robot()

        self.synth_cam = Camera(self.robot_id, camera_config)
        self.lidar = Lidar(self.robot, lidar_config)

        self.lidar.setup()

        self.synth_cam_counter = 0
        self.map_counter = 0
        self.debug_cam_counter = 0

        self.lidar_values = []
        self.robot_values = []

        self.init_debug_sliders()
        self.read_debug_sliders()
        self.init_cube_creator()
        self.store_camera_data()

    def init_ui(self):
        self.setWindowTitle("PyBullet + PyQt6 Simulation")
        self.setGeometry(100, 100, 600, 500)

        # Main layout of the GUI
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # UI elements of the Start/Stop button
        self.toggle_button = QPushButton("Stop Simulation", self)
        self.toggle_button.clicked.connect(self.toggle_simulation)
        layout.addWidget(self.toggle_button)

        # Visualization label
        self.visualization_label = QLabel(self)
        self.visualization_label.setText("Simulation Visualization")
        layout.addWidget(self.visualization_label)

        # Robot map
        self.robot_map = RobotMap(DEFAULT_GRID_SIZE)
        layout.addWidget(self.robot_map)

    def slider_ui(self):
        #Slider setup
        self.slider = QtWidgets.QSlider(Qt.Orientation.Horizontal, self)
        self.slider.setMinimum(0)
        self.slider.setMaximum(50) # Allows users to slide between 0 Lidar rays to 50
        self.slider.setValue(0) # Intializing Lidar to have 20 rays 
        self.slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.slider.setTickInterval(2)
        self.slider.valueChanged.connect(self.on_slide) # Connects to a slot to handle slider

        # Adding the sldier to the layout 
        central_widget = self.centralWidget()
        layout = central_widget.layout()
        layout.addWidget(self.slider)
    
    def on_slide(self,value):
        if value > 0:
            print(f"Slider value {value}") # For debugging slider
            if value != self.lidar.num_rays: #only updating if the value has changed 
                self.lidar.gui_change_parameter(num_rays=value)

        else:
            print("No Lidar Rays")
            # Remove Every single ray if the slider value is 0
            for ray_id in self.lidar.ray_ids:
                p.removeUserDebugItem(ray_id)
            self.lidar.ray_ids.clear()

    def init_debug_sliders(self):
        self.lidar_sliders = []

        self.lidar_sliders.append(
            p.addUserDebugParameter("  number of rays", 0, MAX_RAY_NUMBER, self.lidar.num_rays)
        )
        self.lidar_sliders.append(
            p.addUserDebugParameter("  start angle", 0, MAX_START_ANGLE, self.lidar.start_angle)
        )
        self.lidar_sliders.append(
            p.addUserDebugParameter("  end angle", 0, MAX_END_ANGLE, self.lidar.end_angle)
        )
        self.lidar_sliders.append(
            p.addUserDebugParameter("  ray length", 0, MAX_RAY_LENGTH, self.lidar.ray_len)
        )

        self.robot_sliders = []

        self.robot_sliders.append(
            p.addUserDebugParameter("  max robot velocity", 0, MAX_ROBOT_VELOCITY, self.robot.max_velocity),
        )

        self.robot_sliders.append(
            p.addUserDebugParameter("  robot acceleration", 0, MAX_FORCE, self.robot.force)
        )

    def init_cube_creator(self):
        self.cube_creator = CubeCreator()
        self.cube_creator.create_info_text()

    def read_debug_sliders(self):
        self.old_lidar_param = self.lidar_values
        self.old_robot_param = self.robot_values

        self.lidar_values = {
            "num_rays": int(p.readUserDebugParameter(self.lidar_sliders[0])),
            "start_angle": p.readUserDebugParameter(self.lidar_sliders[1]),
            "end_angle": p.readUserDebugParameter(self.lidar_sliders[2]),
            "ray_len": p.readUserDebugParameter(self.lidar_sliders[3]),
        }

        self.robot_values = {
            "max_velocity": int(p.readUserDebugParameter(self.robot_sliders[0])),
            "force": int(p.readUserDebugParameter(self.robot_sliders[1]))
        }

    def store_camera_data(self):
        camera_state = p.getDebugVisualizerCamera()
        self.cam_target_position = camera_state[11]  # Target position (x, y, z)
        self.cam_distance = camera_state[10]         # Camera distance
        self.cam_yaw = camera_state[8]               # Yaw angle
        self.cam_pitch = camera_state[9]             # Pitch angle

    def update_synth_camera(self):
        self.synth_cam_counter += 1

        # Update camera sensor at specified intervals
        if self.synth_cam_counter == SYNTH_CAMERA_UPDATE_INTERVAL:
            self.synth_cam.update_sensor()
            self.synth_cam_counter = 0

    def update_debug_camera(self):
        self.debug_cam_counter += 1

        if self.debug_cam_counter == DEBUG_CAMERA_UPDATE_INTERVAL:
            robot_pos = self.robot.get_pos()
            p.resetDebugVisualizerCamera(self.cam_distance, self.cam_yaw,
                                         self.cam_pitch, robot_pos)
            self.debug_cam_counter = 0
        

    def toggle_simulation(self):
        if self.simulation_running:
            self.timer.stop()
            self.store_camera_data()
            self.physicsClient = p.disconnect(self.physicsClient)
            self.robot_map.reset_map()
            self.simulation_running = False
            self.toggle_button.setText("Start Simulation")
        else:
            self.timer.start(SIM_TIME_CONSTANT)
            self.init_simulation()
            p.resetDebugVisualizerCamera(self.cam_distance, self.cam_yaw,
                                         self.cam_pitch, self.cam_target_position)
            self.simulation_running = True
            self.toggle_button.setText("Stop Simulation")

    def update_simulation(self):
        # Step PyBullet simulation
        p.stepSimulation()

        # Detect keypress for robot movement
        self.keypress_detection()

        # Read and changes LIDAR data
        self.read_debug_sliders()
        if self.old_lidar_param != self.lidar_values:
            self.lidar.gui_change_parameter(**self.lidar_values)
        
        if self.old_robot_param != self.robot_values:
            self.robot.gui_change_parameter(**self.robot_values)

        # Update LIDAR data
        rays_data, dists, coords  = self.lidar.retrieve_data(common=False)
        robot_pos = self.robot.get_pos()

        # For visualization, create a simple 2D plot (e.g., with QImage/QPixmap)
        """
        lidar_image = np.zeros((480, 640, 3), dtype=np.uint8)
        for i, dist in enumerate(dists):
            if dist < self.lidar.ray_len:
                x = int(320 + dist * np.cos(np.radians(bearings[i])) * 100)
                y = int(240 - dist * np.sin(np.radians(bearings[i])) * 100)
                if 0 <= x < 640 and 0 <= y < 480:
                    lidar_image[y, x] = [255, 0, 0]  # Red for hits
        q_image = QImage(lidar_image, 640, 480, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_image)
        self.visualization_label.setPixmap(pixmap)
        """

        if (self.map_counter == 15):
            ray_len = self.lidar_values["ray_len"]
            yaw = self.robot.get_yaw()
            self.robot_map.third_calculate_matrix(robot_pos, coords,self.lidar_values, yaw, rays_data)
            self.map_counter = 0
        
        self.map_counter += 1
        self.lidar.simulate(rays_data)
        self.view_matrix = self.update_synth_camera()
        self.cube_creator.update_cube_button(robot_pos)
        self.update_debug_camera()

    def keypress_detection(self):
        """
        Detects key presses and moves the robot accordingly.
        """
        keys = p.getKeyboardEvents()

        w_pressed = (ord("w") in keys and keys[ord("w")] & p.KEY_IS_DOWN) or \
                    (p.B3G_UP_ARROW in keys and keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN)

        s_pressed = (ord("s") in keys and keys[ord("s")] & p.KEY_IS_DOWN) or \
                    (p.B3G_DOWN_ARROW in keys and keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN)

        a_pressed = (ord("a") in keys and keys[ord("a")] & p.KEY_IS_DOWN) or \
                    (p.B3G_LEFT_ARROW in keys and keys[p.B3G_LEFT_ARROW] & p.KEY_IS_DOWN)

        d_pressed = (ord("d") in keys and keys[ord("d")] & p.KEY_IS_DOWN) or \
                    (p.B3G_RIGHT_ARROW in keys and keys[p.B3G_RIGHT_ARROW] & p.KEY_IS_DOWN)
        if w_pressed and not s_pressed:
            self.robot.move("W")
        elif s_pressed and not w_pressed:
            self.robot.move("S")
        else:
            self.robot.stop_motion()

        if a_pressed:
            self.robot.move("A")
        elif d_pressed:
            self.robot.move("D")
        else:
            self.robot.stop_angle()

    @staticmethod
    def open_yaml(config_name):
        try:
            with open(SENSOR_CONFIG_PATH, 'r') as file:
                return yaml.safe_load(file)[config_name]
        except FileNotFoundError:
            print("Config file not found!")
            sys.exit(0)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = SimulationApp()
    main_window.show()
    sys.exit(app.exec())