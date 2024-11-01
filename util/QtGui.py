from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QSlider, QTextEdit, QLabel
from PyQt5.QtCore import Qt

class Widget(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout()

        # Sliders for X and Y velocities and goals
        self.x_velocity_slider = QSlider(Qt.Horizontal)
        self.x_velocity_slider.setRange(-10, 10)

        self.y_velocity_slider = QSlider(Qt.Horizontal)
        self.y_velocity_slider.setRange(-10, 10)

        self.x_goal_slider = QSlider(Qt.Horizontal)
        self.x_goal_slider.setRange(-100, 100)

        self.y_goal_slider = QSlider(Qt.Horizontal)
        self.y_goal_slider.setRange(-100, 100)

        # Output box
        self.output_box = QTextEdit()
        self.output_box.setReadOnly(True)

        # Layout for sliders
        slider_layout = QVBoxLayout()

        # Add sliders with labels
        slider_layout.addWidget(QLabel('X Velocity'))
        slider_layout.addWidget(self.x_velocity_slider)
        
        slider_layout.addWidget(QLabel('Y Velocity'))
        slider_layout.addWidget(self.y_velocity_slider)
        
        slider_layout.addWidget(QLabel('X Goal'))
        slider_layout.addWidget(self.x_goal_slider)
        
        slider_layout.addWidget(QLabel('Y Goal'))
        slider_layout.addWidget(self.y_goal_slider)

        
        # # connecting sliders to update
        # self.x_velocity_slider.valueChanged.connect(self.update)
        # self.y_velocity_slider.valueChanged.connect(self.update)
        # self.x_goal_slider.valueChanged.connect(self.update)
        # self.y_goal_slider.valueChanged.connect(self.update)
        
        # Add components to the main layout
        layout.addLayout(slider_layout)
        layout.addWidget(QLabel("Output:"))
        layout.addWidget(self.output_box)

        # Set the layout for this widget
        self.setLayout(layout)
        self.setWindowTitle("Control Panel")

    def update(self):
        try:
            output_text = (f"X Velocity: {self.x_velocity_slider.value()}\n"
                        f"Y Velocity: {self.y_velocity_slider.value()}\n"
                        f"X Goal: {self.x_goal_slider.value()}\n"
                        f"Y Goal: {self.y_goal_slider.value()}")
            self.output_box.setPlainText(output_text)
        except Exception as e:
            print(f"Error in update method: {e}")
            
    def get_values(self):
        self.values={'X_velocity':self.x_velocity_slider.value(),'Y_velocity':self.y_velocity_slider.value(), 'X_goalpos': self.x_goal_slider.value(), 'Y_goalpos': self.y_goal_slider.value()}
        return self.values
