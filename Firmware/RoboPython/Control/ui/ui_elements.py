from PyQt5.QtWidgets import QVBoxLayout, QLabel, QSlider, QHBoxLayout, QWidget, QLineEdit
from PyQt5.QtCore import Qt

def create_slider(label_text, min_val, max_val, init_val):
    layout = QVBoxLayout()
    label = QLabel(f"{label_text}: {init_val}")
    slider = QSlider(Qt.Horizontal)
    slider.setMinimum(min_val)
    slider.setMaximum(max_val)
    slider.setValue(init_val)
    slider.valueChanged.connect(lambda val: label.setText(f"{label_text}: {val}"))
    layout.addWidget(label)
    layout.addWidget(slider)
    return layout, slider, label

def create_labeled_input(label_text, default_value=""):
    layout = QHBoxLayout()
    label = QLabel(label_text)
    input_field = QLineEdit()
    input_field.setText(default_value)
    layout.addWidget(label)
    layout.addWidget(input_field)
    return layout, input_field
