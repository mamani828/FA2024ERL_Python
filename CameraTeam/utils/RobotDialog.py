import csv

from PyQt6.QtCore import Qt
from PyQt6.QtWidgets import (QVBoxLayout, QLabel, QPushButton,
                             QMessageBox, QWidget,
                             QSizePolicy)
from PyQt6.QtGui import QColor, QPalette


class RobotDialog(QWidget):
    def __init__(self, parent=None, csv_file=None):
        super().__init__(parent)

        # Initialize variables
        self.csv_file = csv_file
        self.data = []
        self.current_index = 0

        # Layout and widgets
        self.layout = QVBoxLayout(self)
        self.label = QLabel("Click next to continue to the tutorial!", self)
        self.next_button = QPushButton("Next", self)

        # Add widgets to layout
        self.layout.addWidget(self.label)
        self.layout.addWidget(self.next_button, alignment=Qt.AlignmentFlag.AlignRight)

        # Button connections
        self.load_csv()
        self.next_button.clicked.connect(self.show_next_row)

        # Ensure the button resizes correctly
        self.next_button.setFixedSize(self.width() // 7, 20)
        self.next_button.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)

    def resizeEvent(self, event):
        """
        Override resizeEvent to dynamically update button width.
        """
        super().resizeEvent(event)
        self.next_button.setFixedSize(self.width() // 7, 20)

    def load_csv(self):
        try:
            with open(self.csv_file, "r") as file:
                reader = csv.reader(file)
                self.data = list(reader)
                self.current_index = 0
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load CSV file: {str(e)}")

    def show_next_row(self):
        if not self.data:
            self.label.setText("No data loaded. Please load a CSV file.")
            return

        if self.current_index < len(self.data):
            # Display the current row
            row = self.data[self.current_index]
            self.label.setText(", ".join(row))
            self.current_index += 1
        else:
            self.label.setText("End of tutorial!")

    def set_color(self, background_color, text_color, button_color, button_hover_color):
        # Set the dialog background color
        self.setAutoFillBackground(True)
        dialog_palette = self.palette()
        dialog_palette.setColor(QPalette.ColorRole.Window, QColor(background_color))
        self.setPalette(dialog_palette)

        # Set the text color for the label
        label_palette = self.label.palette()
        label_palette.setColor(QPalette.ColorRole.WindowText, QColor(text_color))
        self.label.setPalette(label_palette)
        self.label.setStyleSheet(f"color: {text_color};")

        # Set button color and hover effect using a stylesheet
        self.next_button.setStyleSheet(f"""
            QPushButton {{
                background-color: {button_color};
                color: {background_color};
                border: none;
                padding: 5px;
                border-radius: 5px;
            }}
            QPushButton:hover {{
                background-color: {button_hover_color};
            }}
        """)
