import sys
from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
from PyQt6.QtGui import QWindow
from PyQt6.QtCore import Qt
import pybullet as p
import pybullet_data

class PyBulletWindow(QWidget):
    def __init__(self, bullet_env, parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.WindowType.Widget)

        # Initialize PyBullet in GUI mode
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Get the window handle for the PyBullet GUI
        debug_visualizer = p.getDebugVisualizerCamera()
        window_id = debug_visualizer.get('windowUniqueId', None)

        if window_id is None:
            raise RuntimeError("Unable to get PyBullet window handle")

        # Embed the PyBullet GUI window into PyQt6
        pybullet_window = QWindow.fromWinId(window_id)
        pybullet_container = QWidget.createWindowContainer(pybullet_window, self)
        
        layout = QVBoxLayout()
        layout.addWidget(pybullet_container)
        self.setLayout(layout)
        
    def closeEvent(self, event):
        # Disconnect PyBullet on close
        p.disconnect(self.physicsClient)
        event.accept()

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("PyQt6 with PyBullet")
        self.setGeometry(100, 100, 800, 600)
        
        self.pybullet_widget = PyBulletWindow(self)
        self.setCentralWidget(self.pybullet_widget)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec())