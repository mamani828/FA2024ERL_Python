import sys
import pybullet as p
import pybullet_data
import numpy as np
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QVBoxLayout, QWidget, QLabel
from PyQt5.QtCore import QTimer
from PyQt5.QtGui import QPixmap, QImage
from Sensors import Camera, Lidar  # Assuming your classes are saved in sensor_code.py
from SensorTest import Robot

class SimulationApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.simulation_running = False

        # Initialize PyBullet simulation
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.robot_id = Robot()

        self.camera = Camera(self.robot_id, {
            'camera_distance': 2,
            'yaw': 0,
            'pitch': -30,
            'roll': 0
        })
        
        self.lidar = Lidar(self.robot_id, {
            'lidar_joints': 0,
            'hit_color': [1, 0, 0],
            'miss_color': [0, 1, 0],
            'num_rays': 20,
            'lidar_angle1': 0,
            'lidar_angle2': 360,
            'ray_start_len': 0.1,
            'ray_len': 5
        })
        self.lidar.setup()

        # Timer for updating the simulation
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_simulation)

    def initUI(self):
        self.setWindowTitle("PyBullet + PyQt5 Simulation")
        self.setGeometry(100, 100, 800, 600)

        # Main layout
        central_widget = QWidget(self)
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Start/Stop button
        self.toggle_button = QPushButton("Start Simulation", self)
        self.toggle_button.clicked.connect(self.toggle_simulation)
        layout.addWidget(self.toggle_button)

        # Visualization label
        self.visualization_label = QLabel(self)
        self.visualization_label.setText("Simulation Visualization")
        self.visualization_label.setFixedSize(640, 480)
        layout.addWidget(self.visualization_label)

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

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = SimulationApp()
    main_window.show()
    sys.exit(app.exec_())