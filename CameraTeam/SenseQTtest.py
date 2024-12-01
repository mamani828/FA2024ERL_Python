import sys
import os
import yaml
import pybullet as p
import pybullet_data
import numpy as np
from PyQt6.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel, QSlider
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QPixmap, QImage
from PyQt6 import QtWidgets
from Sensors import Camera, Lidar
from SensorTest import Robot
from QtMap import RobotMap


SENSOR_CONFIG_PATH = os.path.join(os.path.dirname(__file__),
                                  'config/sensors.yaml')
DEFAULT_GRID_SIZE = 200
SIM_TIME_CONSTANT = 4  # How often the simulation is updated
CAMERA_UPDATE_INTERVAL = 240
MAX_RAY_NUMBER = 75
MAX_START_ANGLE = 360
MAX_END_ANGLE = 360
MAX_RAY_LENGTH = 10

class SimulationApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.init_ui()
        #self.slider_ui()
        self.init_simulation()
        self.init_debug_sliders()
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

        self.robot = Robot()

        camera_config = SimulationApp.open_yaml("camera_configs")
        lidar_config = SimulationApp.open_yaml("lidar_configs")

        self.camera = Camera(self.robot, camera_config)
        self.camera_counter = 0
        
        self.lidar = Lidar(self.robot, lidar_config)
        self.lidar.setup()

        self.counter = 0

    def init_ui(self):
        self.setWindowTitle("PyBullet + PyQt5 Simulation")
        self.setGeometry(100, 100, 800, 600)

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
        self.robot_map.setFixedSize(640, 640)
        layout.addWidget(self.robot_map)
        
        
        # UI elements of the Slider button

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
        self.sliders = []
        self.sliders.append(
            p.addUserDebugParameter("number of rays", 0, MAX_RAY_NUMBER, self.lidar.num_rays)
        )
        self.sliders.append(
            p.addUserDebugParameter("start angle", 0, MAX_START_ANGLE, self.lidar.start_angle)
        )
        self.sliders.append(
            p.addUserDebugParameter("end angle", 0, MAX_END_ANGLE, self.lidar.end_angle)
        )
        self.sliders.append(
            p.addUserDebugParameter("ray length", 0, MAX_RAY_LENGTH, self.lidar.ray_len)
        )

    def read_debug_sliders(self):
        self.gui_values = {
            "num_rays": int(p.readUserDebugParameter(self.sliders[0])),
            "start_angle": p.readUserDebugParameter(self.sliders[1]),
            "end_angle": p.readUserDebugParameter(self.sliders[2]),
            "ray_len": p.readUserDebugParameter(self.sliders[3]),
        }

    def store_camera_data(self):
        camera_state = p.getDebugVisualizerCamera()
        self.cam_target_position = camera_state[11]  # Target position (x, y, z)
        self.cam_distance = camera_state[10]         # Camera distance
        self.cam_yaw = camera_state[8]               # Yaw angle
        self.cam_pitch = camera_state[9]             # Pitch angle

    def toggle_simulation(self):
        if self.simulation_running:
            self.timer.stop()
            self.store_camera_data()
            self.physicsClient = p.disconnect(self.physicsClient)
            self.simulation_running = False
            self.toggle_button.setText("Start Simulation")
        else:
            self.timer.start(SIM_TIME_CONSTANT)
            self.initSim()
            p.resetDebugVisualizerCamera(self.cam_distance, self.cam_yaw,
                                         self.cam_pitch, self.cam_target_position)
            self.simulation_running = True
            self.toggle_button.setText("Stop Simulation")

    def update_camera(self):
        self.camera_counter += 2

            # Update camera sensor at specified intervals
        if self.camera_counter % CAMERA_UPDATE_INTERVAL == 0:
                view_matrix = self.camera.update_sensor()
                width, height, rgb, depth, seg = p.getCameraImage(640, 480, viewMatrix=view_matrix)  # Lower resolution

    def update_simulation(self):
        # Step PyBullet simulation
        p.stepSimulation()
        
        # Update camera data
        view_matrix = self.camera.update_sensor()

        # Update LiDAR data
        rays_data, dists, coords  = self.lidar.retrieve_data(common=False)
        robot_pos, robot_orn = p.getBasePositionAndOrientation(self.robot.robot_id)

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

        if (self.counter == 240):
            self.robot_map.calculate_matrix(robot_pos, coords)
            self.counter = 0
        
        self.counter += 1

        self.read_debug_sliders()
        self.lidar.gui_change_parameter(**self.gui_values)
        self.update_camera()

    @staticmethod
    def open_yaml(config_name):
        try:
            with open(SENSOR_CONFIG_PATH, 'r') as file:
                return yaml.safe_load(file)[config_name]
        except FileNotFoundError:
            print("Config file not found!")
            sys.exit(0)

class Object:
    def __init__(self, coordinates, color):
        self.color = color
        self.coordinates = coordinates
        self.object_id = self.loading_box()  # Storing the object's ID

    def loading_box(self):
        half_extents = [0.5, 0.5, 0.5]
        box_coordinates = self.coordinates
        box_orientation = [0, 0, 0]
        initial_box_orientation = p.getQuaternionFromEuler(box_orientation)

        box_collision_shape = p.createCollisionShape(shapeType=p.GEOM_BOX, halfExtents=half_extents)
        box_visual_shape = p.createVisualShape(shapeType=p.GEOM_BOX, halfExtents=half_extents, rgbaColor=self.color)

        # Create a multibody and return its ID
        return p.createMultiBody(baseMass=50, baseCollisionShapeIndex=box_collision_shape,
                                 baseVisualShapeIndex=box_visual_shape,
                                 basePosition=box_coordinates,
                                 baseOrientation=initial_box_orientation)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = SimulationApp()
    main_window.show()
    sys.exit(app.exec())