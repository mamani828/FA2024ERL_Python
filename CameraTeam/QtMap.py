import sys
import numpy as np
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget
from PyQt6.QtGui import QPainter, QColor
from PyQt6.QtCore import Qt, QTimer



COLOR_MAP = {0: QColor("white"), 1: QColor("black"),
             2: QColor("red"), 3: QColor("blue")}
DEFAULT_SCALE = 2  # Scale from PyBullet to map
CELL_SIZE = 3  # Pixel length/width of each cell
WHITE = 0
BLACK = 1
RED = 2
BLUE = 3


class RobotMap(QWidget):
    def __init__(self, grid_size):
        super().__init__()
        self.cell_size = CELL_SIZE
        self.grid = np.zeros((grid_size, grid_size), dtype=int)
        self.grid_size = grid_size  # Number of cells along each dimension
        self.color_map = COLOR_MAP
        self.robot_x = 0
        self.robot_y = 0

        # Set the widget size based on the grid dimensions
        self.setFixedSize(self.grid_size * self.cell_size // 2, self.grid_size * self.cell_size // 2)

    def update_map(self):
        """Update the grid and trigger a repaint."""
        self.update()

    def calculate_matrix(self, robot_pos, ray_pos, distance=None, scaling_factor=DEFAULT_SCALE):
        #  Sets previous robot position to white
        robot_grid_x = int(self.robot_x // scaling_factor)
        robot_grid_y = int(self.robot_y // scaling_factor)
        self.grid[robot_grid_x][robot_grid_y] = WHITE

        #  Gets new robot position and sets color in map
        self.robot_x, self.robot_y, _ = robot_pos
        robot_grid_x = int(self.robot_x // scaling_factor)
        robot_grid_y = int(self.robot_y // scaling_factor)
        self.grid[robot_grid_x][robot_grid_y] = RED

        #  Iterates through ray data and sets position to black
        for ray_x, ray_y in ray_pos:
            if np.isnan(ray_x) or np.isnan(ray_y):
                continue
            else:
                grid_x = ray_x // scaling_factor
                grid_y = ray_y // scaling_factor
                if (grid_x == robot_grid_x and grid_y == robot_grid_y):
                    continue
                self.grid[int(grid_x)][int(grid_y)] = BLACK
        self.update_map()
        

    def paintEvent(self, event):
        painter = QPainter(self)
        for row in range(self.grid_size):
            for col in range(self.grid_size):
                # Set the brush color based on the matrix value
                painter.setBrush(self.color_map[self.grid[row][col]])
                painter.setPen(Qt.PenStyle.NoPen)  # Optional: remove grid lines
                # Draw the cell rectangle
                painter.drawRect(
                    col * self.cell_size,
                    row * self.cell_size,
                    self.cell_size,
                    self.cell_size,
                )
