import sys
import numpy as np
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget
from PyQt6.QtGui import QPainter, QColor
from PyQt6.QtCore import Qt, QTimer

class RobotMap(QWidget):
    def __init__(self, grid):
        super().__init__()
        self.cell_size = 3  # Size of each cell in pixels
        self.grid = grid  # Matrix (numpy array or list of lists)
        self.grid_size = len(grid)  # Number of cells along each dimension
        self.color_map = {0: QColor("white"), 1: QColor("black"), 2: QColor("red")}

        # Set the widget size based on the grid dimensions
        self.setFixedSize(self.grid_size * self.cell_size, self.grid_size * self.cell_size)

    def update_grid(self, new_grid):
        """Update the grid and trigger a repaint."""
        self.grid = new_grid
        self.update()

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

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Real-Time Robot Map")
        self.setGeometry(100, 100, 600, 600)

        # Initialize a sample 100x100 matrix
        self.grid_size = 200
        self.matrix = self.generate_sample_matrix(self.grid_size)

        # Create an instance of RobotMap and set it as the central widget
        self.robot_map = RobotMap(self.matrix)
        self.setCentralWidget(self.robot_map)

        # Set up a QTimer to update the map in real time
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_map)  # Connect to update method
        self.timer.start(500)  # Update every 500 milliseconds

    def generate_sample_matrix(self, size):
        """Generate a random matrix with values 0, 1, or 2."""
        return np.random.choice([0, 1, 2], size=(size, size))

    def update_map(self):
        """Generate a new matrix and update the map."""
        self.matrix = self.generate_sample_matrix(self.grid_size)
        self.robot_map.update_grid(self.matrix)

# Entry point of the application
if __name__ == "__main__":
    app = QApplication(sys.argv)

    # Create and show the main window
    main_window = MainWindow()
    main_window.show()

    sys.exit(app.exec())