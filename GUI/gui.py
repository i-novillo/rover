from PySide6.QtWidgets import *
from PySide6.QtGui import *
from PySide6.QtCore import *
from widgets import WASD
from bt_client import BTClient

KEY_WIDTH = 50
KEY_HEIGHT = 50
KEY_UNPRESSED_COLOR = QColor(0, 0, 255)

class Window(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.setWindowTitle("Keyboard")
        self.setGeometry(100, 100, 1500, 750)
        self.setStyleSheet("background-color: white;")

        print("Awaiting for successful bluetooth connection to the rover")
        self.bt_client = BTClient()
        print("Bluetooth connection to the rover successful")

        self.wasd = WASD(self.bt_client, parent=self)
        self.wasd.setGeometry(100, 100, 600, 800)

        self.installEventFilter(self.wasd.eventFilter)


if __name__ == '__main__':
    app = QApplication([])
    window = Window()
    window.show()
    app.exec()