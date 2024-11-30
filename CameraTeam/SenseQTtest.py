import sys
import os
import yaml
import pybullet as p
import pybullet_data
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QPixmap, QImage
from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QSlider
from PyQt5.QtCore import Qt
from Sensors import Camera, Lidar
from SensorTest import Robot


SENSOR_CONFIG_PATH = os.path.join(os.path.dirname(__file__),
                                  'config/sensors.yaml')




class SimulationApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.sliderUI()
        self.simulation_running = False

        # Initialize PyBullet simulation
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot = Robot()

        camera_config = SimulationApp.open_yaml("camera_configs")
        lidar_config = SimulationApp.open_yaml("lidar_configs")

        self.camera = Camera(self.robot, camera_config)
        
        self.lidar = Lidar(self.robot, lidar_config)
        self.lidar.setup()

        # Timer for updating the simulation
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_simulation)

    def initUI(self):
        self.setWindowTitle("PyBullet + PyQt5 Simulation")
        self.setGeometry(100, 100, 800, 600)

        # Main layout of the GUI
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # UI elements of the Start/Stop button
        self.toggle_button = QPushButton("Start Simulation", self)
        self.toggle_button.clicked.connect(self.toggle_simulation)
        layout.addWidget(self.toggle_button)

        # Visualization label
        self.visualization_label = QLabel(self)
        self.visualization_label.setText("Simulation Visualization")
        self.visualization_label.setFixedSize(640, 480)
        layout.addWidget(self.visualization_label)
        
        # UI elements of the Slider button

    def sliderUI(self):
        #Slider setup
        self.slider = QtWidgets.QSlider(Qt.Horizontal, self)
        self.slider.setMinimum(0)
        self.slider.setMaximum(50) # Allows users to slide between 0 Lidar rays to 50
        self.slider.setValue(0) # Intializing Lidar to have 20 rays 
        self.slider.setTickPosition(QSlider.TicksBelow)
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

    def toggle_simulation(self):
        if self.simulation_running:
            self.timer.stop()
            p.disconnect(self.physicsClient)
            self.simulation_running = False
            self.toggle_button.setText("Start Simulation")
        else:
            self.physicsClient = p.connect(p.GUI)
            self.timer.start(100)  # Update every 100ms
            self.simulation_running = True
            self.toggle_button.setText("Stop Simulation")

    def update_simulation(self):
        # Step PyBullet simulation
        p.stepSimulation()
        
        # Update camera data
        view_matrix = self.camera.update_sensor()

        # Update LiDAR data
        rays_data, dists, bearings = self.lidar.retrieve_data(robot_state=[0, 0, 0])

        # For visualization, create a simple 2D plot (e.g., with QImage/QPixmap)
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
    sys.exit(app.exec_())