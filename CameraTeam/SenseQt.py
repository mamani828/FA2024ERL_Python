from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPushButton, QSlider, QTextEdit, QLabel
from PyQt5.QtCore import Qt

class Widget(QWidget):
    def __init__(self):
        super().__init__()
        layout = QVBoxLayout()