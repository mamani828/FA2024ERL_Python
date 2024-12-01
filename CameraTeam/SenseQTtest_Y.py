import sys
import os
import yaml
import pybullet as p
import pybullet_data
import numpy as np
from PyQt6.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel, QSlider
from PyQt6.QtCore import QTimer, Qt
from PyQt6.QtGui import QPixmap, QImage, QWindow
from PyQt6 import QtWidgets

from Quartz import CGWindowListCopyWindowInfo, kCGWindowListOptionOnScreenOnly
from AppKit import NSApplication, NSRunningApplication
from PyQt5 import sip

from utils.Sensors import Camera, Lidar
from utils.QtMap import RobotMap
from utils.SimulationObjects import Robot, Object


SENSOR_CONFIG_PATH = os.path.join(os.path.dirname(__file__),
                                  'config/sensors.yaml')
DEFAULT_GRID_SIZE = 10
SIM_TIME_CONSTANT = 4  # How often the simulation is updated


class SimulationApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.sliderUI()
        self.initSim()
        #self.initMap()
        self.simulation_running = True

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_simulation)
        self.timer.start(SIM_TIME_CONSTANT)

    def initSim(self):
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
        
        self.lidar = Lidar(self.robot, lidar_config)
        self.lidar.setup()

        self.counter = 0

    def initUI(self):
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
        
        # Look for the PyBullet window
        window_id = self.find_pybullet_window()
        #window = self.get_nswindow_from_kcgwindownumber(window_id)
        window = self.get_window_pointer(window_id)

        # Embed the PyBullet GUI window into PyQt5
        pybullet_window = QWindow.fromWinId(window_id)
        pybullet_container = QWidget.createWindowContainer(pybullet_window, self)
        
        layout.addWidget(pybullet_container)
        self.setLayout(layout)
        
        # UI elements of the Slider button

    def sliderUI(self):
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
                for ray_id in self.lidar.ray_ids:
                    p.removeUserDebugItem(ray_id)
                
                self.lidar.gui_change_parameter(num_rays=value)

        else:
            print("No Lidar Rays")
            # Remove Every single ray if the slider value is 0
            for ray_id in self.lidar.ray_ids:
                p.removeUserDebugItem(ray_id)
            self.lidar.ray_ids.clear()
            
    def get_window_pointer(self, ns_window):
        """
        Convert NSWindow to a pointer usable with QWindow.fromWinId.
        """
        if ns_window:
            return sip.voidptr(ns_window)
        return None
        
    def get_nswindow_from_kcgwindownumber(self, kcg_window_number):
        # Get the list of all running applications
        app = NSApplication.sharedApplication()
        running_apps = NSRunningApplication.runningApplicationsWithBundleIdentifier_("org.python.python")
        
        for window in app.orderedWindows():
            if window.windowNumber() == kcg_window_number:
                return window
        return None
    
    def find_pybullet_window(self):
        """
        Searches for the PyBullet window and returns its window ID.
        """
        # Get all on-screen windows
        windows = CGWindowListCopyWindowInfo(kCGWindowListOptionOnScreenOnly, 0)
        
        # Look for the PyBullet window
        for window in windows:
            if "Bullet Physics" in window.get('kCGWindowName', ''):
                print(f"Found PyBullet Window! ID: {window.get('kCGWindowNumber')}")
                return window.get('kCGWindowNumber')
        return 0  # Return 0 if no window is found

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