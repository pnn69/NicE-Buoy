from PyQt5.QtCore import Qt, QRectF, QTimer
from PyQt5.QtWidgets import QWidget, QApplication, QVBoxLayout, QHBoxLayout, QLabel
from PyQt5.QtGui import QPainter, QColor, QBrush, QPen
import sys


class VerticalCenteredProgressBar(QWidget):
    def __init__(self, width=30, height=150, parent=None):
        super().__init__(parent)
        self._value = 0
        self.setFixedSize(width, height)

    def setValue(self, value):
        self._value = max(-100, min(100, value))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        rect = self.rect()
        w = rect.width()
        h = rect.height()

        painter.setBrush(QBrush(Qt.lightGray))
        painter.setPen(QPen(Qt.darkGray))
        painter.drawRect(rect)

        mid_y = h / 2
        if self._value == 0:
            return

        length = (abs(self._value) / 100) * (h / 2)

        if self._value > 0:
            fill_rect = QRectF(0, mid_y - length, w, length)
            color = QColor(0, 180, 0)
        else:
            fill_rect = QRectF(0, mid_y, w, length)
            color = QColor(180, 0, 0)

        painter.setBrush(QBrush(color))
        painter.setPen(Qt.NoPen)
        painter.drawRect(fill_rect)


class DoubleBarWidget(QWidget):
    def __init__(self):
        super().__init__()

        self.bb_bar = VerticalCenteredProgressBar()
        self.sb_bar = VerticalCenteredProgressBar()

        bb_label = QLabel("BB")
        bb_label.setAlignment(Qt.AlignCenter)
        sb_label = QLabel("SB")
        sb_label.setAlignment(Qt.AlignCenter)

        bb_layout = QVBoxLayout()
        bb_layout.addWidget(self.bb_bar)
        bb_layout.addWidget(bb_label)

        sb_layout = QVBoxLayout()
        sb_layout.addWidget(self.sb_bar)
        sb_layout.addWidget(sb_label)

        main_layout = QHBoxLayout()
        main_layout.addLayout(bb_layout)
        main_layout.addSpacing(20)
        main_layout.addLayout(sb_layout)
        main_layout.addStretch()

        self.setLayout(main_layout)
        self.setWindowTitle("BB and SB Vertical Bars")

        self.value_bb = -100
        self.value_sb = 100

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_bars)
        self.timer.start(100)

    def update_bars(self):
        self.bb_bar.setValue(self.value_bb)
        self.sb_bar.setValue(self.value_sb)

        self.value_bb += 5
        if self.value_bb > 100:
            self.value_bb = -100

        self.value_sb -= 5
        if self.value_sb < -100:
            self.value_sb = 100


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = DoubleBarWidget()
    w.show()
    sys.exit(app.exec_())
