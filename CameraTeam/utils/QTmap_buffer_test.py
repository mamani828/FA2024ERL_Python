import sys
from PyQt6.QtWidgets import QApplication, QWidget
from PyQt6.QtGui import QPainter, QPixmap, QColor
from PyQt6.QtCore import QRect, QTimer

class BitmapWidget(QWidget):
    def __init__(self, rows, cols, cell_size):
        super().__init__()
        self.rows = rows
        self.cols = cols
        self.cell_size = cell_size
        
        # Create a QPixmap to act as the off-screen buffer
        self.pixmap = QPixmap(cols * cell_size, rows * cell_size)
        self.pixmap.fill(QColor('gray'))  # Initialize with a white background

        # Initialize the cell update parameters
        self.current_row = 0
        self.current_col = 0
        self.colors = ['red', 'blue', 'green', 'yellow', 'purple']  # Colors to cycle through

        # Set up a QTimer for real-time updates
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_next_cell)
        self.timer.start(500)  # Update every 500 ms (0.5 seconds)

    def update_cell(self, row, col, color):
        """
        Update a single cell in the pixmap with a new color.
        """
        painter = QPainter(self.pixmap)
        painter.fillRect(
            QRect(col * self.cell_size, row * self.cell_size, self.cell_size, self.cell_size),
            QColor(color)
        )
        painter.end()
        self.update()  # Request a repaint

    def update_next_cell(self):
        """
        Update the next cell in the grid with a color, cycling through rows and columns.
        """
        color = self.colors[(self.current_row + self.current_col) % len(self.colors)]
        self.update_cell(self.current_row, self.current_col, color)
        
        # Move to the next cell
        self.current_col += 1
        if self.current_col >= self.cols:
            self.current_col = 0
            self.current_row += 1
            if self.current_row >= self.rows:
                self.current_row = 0  # Reset to the top row

    def paintEvent(self, event):
        """
        Draw the entire pixmap to the widget.
        """
        painter = QPainter(self)
        painter.drawPixmap(0, 0, self.pixmap)

def main():
    app = QApplication(sys.argv)
    
    # Configuration for the grid
    rows, cols, cell_size = 10, 10, 50
    
    window = BitmapWidget(rows, cols, cell_size)
    window.resize(cols * cell_size, rows * cell_size)
    window.show()
    
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
