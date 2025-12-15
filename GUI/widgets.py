from PySide6.QtWidgets import QWidget
from PySide6.QtGui import QPainter, QColor, QPen, QBrush
from PySide6.QtCore import QEvent, QObject, Qt, QRect

KEY_WIDTH = 50
KEY_HEIGHT = 50
KEY_UNPRESSED_COLOR = QColor(0, 0, 255)

class WASD(QWidget):
    def __init__(self, bt_client, parent=None):
        super().__init__(parent=parent)
        
        self.bt_client = bt_client

        self.keys = [
            self.KeyboardKey('A', 50, 70),
            self.KeyboardKey('S', 120, 70),
            self.KeyboardKey('D', 190, 70),
            self.KeyboardKey('W', 120, 0),
        ]

        self.key_commands = {
            Qt.Key_W: "forward",
            Qt.Key_A: "left",
            Qt.Key_S: "backward",
            Qt.Key_D: "right"
        }

        self.eventFilter = self.KeyPressFilter(parent=self, window=self)

    def paintEvent(self, event):
        super().paintEvent(event)

        painter = QPainter(self)

        for key_obj in self.keys:
            key_rect = QRect(key_obj.x_pos, key_obj.y_pos, KEY_WIDTH, KEY_HEIGHT)
            pen = QPen(QColor(0, 0, 0))
            brush = QBrush(key_obj.color)
            painter.setPen(pen)
            painter.setBrush(brush)
            painter.drawRect(key_rect)

    class KeyboardKey:
        def __init__(self, key, x_pos, y_pos):
            self.key = key
            self.x_pos = x_pos
            self.y_pos = y_pos
            self.color = KEY_UNPRESSED_COLOR

        def set_color(self, color):
            self.color = color

    class KeyPressFilter(QObject):
        def __init__(self, parent=None, window=None):
            super().__init__(parent)
            self.window = window
            self.key_pressed = {}
            self.original_colors = {key_obj.key: KEY_UNPRESSED_COLOR for key_obj in self.window.keys}

        def eventFilter(self, widget, event):
            if event.type() == QEvent.KeyPress:
                if event.isAutoRepeat():
                    return True    
                key = event.key()

                if key in self.window.key_commands:
                    action = self.window.key_commands[key]
                    if key not in self.key_pressed or not self.key_pressed[key]:
                        self.key_pressed[key] = True
                        # DEBUG
                        # print(f"Key pressed: {action}")
                        self.window.bt_client.send_move_cmd(action)
                        for key_obj in self.window.keys:
                            if key_obj.key == self.get_key_char(key):
                                key_obj.set_color(QColor(255, 0, 0))
                                self.window.update()
                                break
                        return True

            elif event.type() == QEvent.KeyRelease:
                if event.isAutoRepeat():
                    return True
                key = event.key()

                if key in self.window.key_commands:
                    action = self.window.key_commands[key]
                    if key in self.key_pressed:
                        self.key_pressed[key] = False
                        if True not in self.key_pressed.values():
                            self.window.bt_client.send_move_cmd("stop")
                        for key_obj in self.window.keys:
                            if key_obj.key == self.get_key_char(key):
                                key_obj.set_color(self.original_colors[key_obj.key])
                                self.window.update()
                                break
                        return True

            return False

        def get_key_char(self, key_code):
            if key_code == Qt.Key_W:
                return 'W'
            elif key_code == Qt.Key_A:
                return 'A'
            elif key_code == Qt.Key_S:
                return 'S'
            elif key_code == Qt.Key_D:
                return 'D'
            return ''