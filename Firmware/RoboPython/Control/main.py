from PyQt5.QtWidgets import QApplication
from widgets.pid_sender import PIDSender
import sys

def main():
    app = QApplication(sys.argv)
    window = PIDSender()
    window.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
