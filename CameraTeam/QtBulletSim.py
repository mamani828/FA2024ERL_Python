import sys
import os
import yaml

import pybullet as p
import pybullet_data

from PyQt6.QtWidgets import (QApplication,
                             QMainWindow,
                             QPushButton,
                             QVBoxLayout,
                             QWidget,
                             QLabel
                             )
from PyQt6.QtCore import QTimer, Qt

from utils.Sensors import Camera, Lidar
from utils.QtMap import RobotMap
from utils.SimulationObjects import Robot, CubeCreator
from utils.RobotDialog import RobotDialog

SENSOR_CONFIG_PATH = os.path.join(os.path.dirname(__file__),
                                  'config/sensors.yaml')
TUTORIAL_CSV_PATH = os.path.join(os.path.dirname(__file__),
                                 'utils/script.csv')
DEFAULT_GRID_SIZE = 100
SIM_TIME_CONSTANT = 4  # How often the simulation is updated
MAP_UPDATE_INTERVAL = 15
SYNTH_CAMERA_UPDATE_INTERVAL = 30
DEBUG_CAMERA_UPDATE_INTERVAL = 30
MAX_RAY_NUMBER = 75
MAX_START_ANGLE = 360
MAX_END_ANGLE = 360
MAX_RAY_LENGTH = 10
MAX_ROBOT_VELOCITY = 100
MAX_FORCE = 2
WALL_MASS = 0  # Mass of 0 to make the wall static
WALL_COLOR = [0.7, 0.7, 0.7, 1]  # Gray color for the wall




class SimulationApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.init_simulation()
        self.store_camera_data()
        self.simulation_running = True

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_simulation)
        self.timer.start(SIM_TIME_CONSTANT)

    def init_simulation(self):
        self.physicsClient = p.connect(p.GUI)
        p.setGravity(0, 0, -9.81)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.objectlist =[] #obj list for 2nd ogm method
        coordinates = [0, 0, 0]  # initial coordinates and orientation for the plane
        orientation = [0, 0, 0]  # Euler Angles Roll, Pitch, Yaw
        initial_orientation = p.getQuaternionFromEuler(orientation)  # Converting to Quaternion
        self.plane_id = p.loadURDF("plane.urdf", coordinates, initial_orientation)

        # Now, call the create_wall method on that instance
        WALL_SIZE = [0.2, 5, 1]  # Wall dimensions (length, width, height)
        wall1 = CubeCreator.create_wall(position=[2, 0, 0], WALL_SIZE=WALL_SIZE, WALL_COLOR=WALL_COLOR)
        wall2 = CubeCreator.create_wall(position=[-2, 0, 0], WALL_SIZE=WALL_SIZE, WALL_COLOR=WALL_COLOR)

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
        self.map_version = 2
        self.debug_cam_counter = 0

        self.lidar_values = []
        self.robot_values = []

        self.init_debug_sliders()
        self.read_debug_sliders()
        self.init_cube_creator()

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
        self.map_version_names = ["OGM Version 3", "OGM Version 1", "OGM Version 2"]
        self.visualization_label.setText("OGM Version 1")
        layout.addWidget(self.visualization_label, alignment=Qt.AlignmentFlag.AlignHCenter)

        # Robot map
        self.robot_map = RobotMap(DEFAULT_GRID_SIZE)
        layout.addWidget(self.robot_map, alignment=Qt.AlignmentFlag.AlignHCenter)

        self.map_version_button = QPushButton("Change Map Version", self)
        self.map_version_button.clicked.connect(self.change_map_version)
        layout.addWidget(self.map_version_button)

        self.csv_dialog = RobotDialog(self, TUTORIAL_CSV_PATH)
        self.csv_dialog.setFixedHeight(75)
        self.csv_dialog.setFixedWidth(600)
        self.csv_dialog.set_color("white", "black", "#252625", "#525452")
        layout.addWidget(self.csv_dialog)

    def change_map_version(self):
        self.map_version += 1
        self.robot_map.reset_map()
        self.visualization_label.setText(self.map_version_names[self.map_version % 3])

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
            p.addUserDebugParameter("  ray length", self.lidar.START_LEN, MAX_RAY_LENGTH, self.lidar.ray_len)
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

    def gui_update_map(self, robot_pos, rays_data, coords):
        ray_len = self.lidar_values["ray_len"]
        yaw = self.robot.get_yaw()
        if self.map_version % 4 == 1:
            self.robot_map.first_calculate_matrix(robot_pos, coords)
        if self.map_version % 4 == 2:
            self.objectlist = self.robot_map.second_calculate_matrix(robot_pos, coords,
                                                                     self.lidar_values,
                                                                     yaw, rays_data,
                                                                     self.objectlist)
        if self.map_version % 4 == 3:
            self.robot_map.third_calculate_matrix(robot_pos, coords, self.lidar_values, yaw, rays_data)
        if self.map_version % 4 == 0:
            self.robot_map.fourth_calculate_matrix(robot_pos, coords, self.lidar_values, yaw, rays_data)
        self.map_counter = 0

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

        if self.map_counter == MAP_UPDATE_INTERVAL:
            self.gui_update_map(robot_pos, rays_data, coords)
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
