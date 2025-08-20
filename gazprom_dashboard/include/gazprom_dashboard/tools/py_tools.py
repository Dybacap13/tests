from PyQt5.QtWidgets import QPlainTextEdit, QFrame
from PyQt5.QtGui import QColor, QTextCharFormat, QTextCursor, QPainter, QPen, QBrush
from PyQt5.QtCore import Qt
import datetime

class ColoredLogger(QPlainTextEdit):
    def __init__(self):
        super().__init__()
        self.setReadOnly(True)

        # Предопределим форматы
        self.formats = {
            "INFO": self.create_format(QColor("white")),
            "WARNING": self.create_format(QColor("yellow")),
            "ERROR": self.create_format(QColor("red")),
            "DEBUG": self.create_format(QColor("gray")),
            "": self.create_format(QColor("white")),
        }

    def create_format(self, color):
        fmt = QTextCharFormat()
        fmt.setForeground(color)
        return fmt

    def add_log(self, level, message):
        time_str = datetime.datetime.now().strftime("%H:%M:%S")
        full_msg = f"{level} [{time_str}]:  {message}"

        # Получаем курсор и применяем форматирование
        cursor = self.textCursor()
        cursor.movePosition(QTextCursor.End)

        # Устанавливаем формат для уровня логирования
        cursor.insertText(f"{level} [{time_str}]: ", self.formats[level])

        # Остальной текст - стандартный цвет (белый)
        cursor.insertText(f"{message}\n", self.formats["INFO"])

        # Прокручиваем к новому сообщению
        self.ensureCursorVisible()


class Circle(QFrame):
    def __init__(self, color_, parent=None):
        super().__init__(parent)
        self.color = color_

    def set_color(self, color):
        self.color = color
        self.update()  # Перерисовываем

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)  # Сглаживание
        painter.setPen(QPen(Qt.NoPen))
        painter.setBrush(QBrush(self.color))

        rect = self.rect()
        painter.drawEllipse(rect)

