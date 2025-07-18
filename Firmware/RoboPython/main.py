import sys
from PyQt5.QtWidgets import QApplication
from RoboGui import PIDSender  # your GUI class

com_port = "COM3"  # example, set your actual port


def main():
    app = QApplication(sys.argv)     # Create QApplication once
    window = PIDSender(com_port=com_port)  # Instantiate your window
    window.show()                    # Show the window!
    sys.exit(app.exec_())            # Start the Qt event loop


if __name__ == "__main__":
    main()
