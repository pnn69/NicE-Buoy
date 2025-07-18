from PyQt5.QtCore import Qt, QRect
from PyQt5.QtGui import QPainter, QPen, QBrush
from PyQt5.QtWidgets import QWidget


class CompassDisplay(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 300)

        self.angle = 0  # compass pointer angle in degrees
        self.speed = 0  # speed percentage (0-100)
        self.speed_max = 100  # max speed for scaling bars

    def set_angle(self, angle):
        self.angle = angle % 360
        self.update()

    def set_speed(self, speed):
        self.speed = max(0, min(speed, self.speed_max))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        rect = self.rect()
        center = rect.center()
        radius = min(rect.width(), rect.height()) // 3

        # Draw compass circle
        painter.setPen(QPen(Qt.black, 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawEllipse(center, radius, radius)

        # Draw compass needle (red line pointing to angle)
        painter.setPen(QPen(Qt.red, 3))
        painter.translate(center)
        painter.rotate(self.angle)
        painter.drawLine(0, 0, 0, -radius + 10)
        painter.resetTransform()

        # Draw speed bar background and fill
        bar_width = 30
        bar_height = radius * 2
        bar_rect = QRect(center.x() + radius + 20, center.y() - radius, bar_width, bar_height)

        painter.setPen(QPen(Qt.black, 2))
        painter.setBrush(Qt.NoBrush)
        painter.drawRect(bar_rect)

        # Fill bar proportional to speed
        fill_height = int(bar_height * self.speed / self.speed_max)
        fill_rect = QRect(bar_rect.x() + 1,
                          bar_rect.y() + bar_height - fill_height + 1,
                          bar_width - 2,
                          fill_height - 2)
        painter.setBrush(QBrush(Qt.green))
        painter.drawRect(fill_rect)

        # Draw speed text
        painter.setPen(Qt.black)
        painter.drawText(bar_rect, Qt.AlignCenter, f"Speed\n{self.speed}%")

if __name__ == "__main__":
    window = CompassDisplay()
    window.show()

    # Add pointers example
    window.draw_pointer(45, "green")
    window.draw_pointer(90, "blue")
    window.draw_pointer(135, "red")

    # Set some example speeds
