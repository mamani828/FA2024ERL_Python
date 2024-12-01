import sys
import numpy as np
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget
from PyQt6.QtGui import QPainter, QColor
from PyQt6.QtCore import Qt, QTimer


COLOR_MAP = {0: QColor("white"), 1: QColor("black"),
             2: QColor("red"), 3: QColor("blue")}
DEFAULT_SCALE = 0.5  # Scale from PyBullet to map
CELL_SIZE = 10  # Pixel length/width of each cell
WHITE = 0
BLACK = 1
RED = 2
BLUE = 3


class RobotMap(QWidget):
    def __init__(self, grid_size):
        super().__init__()
        # Constants
        self.GRID_SIZE = grid_size
        self.OFFSET = grid_size // 4

        self.grid = np.zeros((grid_size, grid_size), dtype=int)
        self.robot_x = 0
        self.robot_y = 0

        # Set the widget size based on the grid dimensions
        self.setFixedSize(self.GRID_SIZE * CELL_SIZE // 2, self.GRID_SIZE * CELL_SIZE // 2)

    def update_map(self):
        """Update the grid and trigger a repaint."""
        self.update()

    def first_calculate_matrix(self, robot_pos, ray_pos, distance=None, scaling_factor=DEFAULT_SCALE):
        #  Sets previous robot position to white
        prev_robot_grid_x = int(self.robot_x // scaling_factor) + self.OFFSET
        prev_robot_grid_y = int(self.robot_y // scaling_factor) + self.OFFSET
        self.grid[prev_robot_grid_x][prev_robot_grid_y] = WHITE

        # Get new robot position and set color in map
        self.robot_x, self.robot_y, _ = robot_pos
        robot_grid_x = int(self.robot_x // scaling_factor) + self.OFFSET
        robot_grid_y = int(self.robot_y // scaling_factor) + self.OFFSET
        self.grid[robot_grid_x][robot_grid_y] = RED

        #  Iterates through ray data and sets position to black
        for ray_x, ray_y in ray_pos:
            if np.isnan(ray_x) or np.isnan(ray_y):
                continue
            else:
                grid_x = int(ray_x // scaling_factor) + self.OFFSET
                grid_y = int(ray_y // scaling_factor) + self.OFFSET
                if (grid_x == robot_grid_x and grid_y == robot_grid_y):
                    continue
                self.grid[int(grid_x)][int(grid_y)] = BLACK

        self.update_map()

    def second_calculate_matrix(self, robot_pos, ray_pos, distance, scaling_factor=DEFAULT_SCALE):
        # Accounts for moving objects
        pass

    def third_calculate_matrix(self, robot_pos, ray_pos, distance, scaling_factor=DEFAULT_SCALE):
        # Uses bressenham
        pass

    def paintEvent(self, event):
        painter = QPainter(self)
        for row in range(self.GRID_SIZE):
            for col in range(self.GRID_SIZE):
                # Set the brush color based on the matrix value
                painter.setBrush(COLOR_MAP[self.grid[row][col]])
                painter.setPen(Qt.PenStyle.NoPen)  # Optional: remove grid lines
                # Draw the cell rectangle
                painter.drawRect(
                    col * CELL_SIZE,
                    row * CELL_SIZE,
                    CELL_SIZE,
                    CELL_SIZE,
                )

    def reset_map(self):
        self.grid = np.zeros((self.GRID_SIZE, self.GRID_SIZE), dtype=int)
        self.update_map()