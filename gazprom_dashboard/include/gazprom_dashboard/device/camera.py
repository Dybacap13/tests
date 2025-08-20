from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtGui import QImage
import enum
import threading
import cv2
import numpy as np

# from #logger.#logger_config import #logger


# *******************************
#            Камера            *
# Флаг остановки камеры        *
# Флаг на сделать фото         *
# *******************************

class StatusCamera(enum.Enum):
    CONNECTED = 0
    ERROR = 1
    NOT_LOADED = 2
    WAITING = 3


class ThreadOpenCV(QThread):
    changePixmap = pyqtSignal(QImage)
    status_connected_camera = StatusCamera.NOT_LOADED

    def __init__(self, port_):

        super().__init__()
        self.port_camera_ = port_
        self.flag_stop_camera = False

        self.flag_foto = False
        self.filename = ""

        self.mutex_save_foto = threading.Lock()
        self.mutex_stop_camera = threading.Lock()
 

    def run(self):
        cap = cv2.VideoCapture(int(self.port_camera_), cv2.CAP_DSHOW)
        cap.set(cv2.CAP_PROP_FPS, 24)
        if not cap.isOpened():
            self.status_connected_camera = StatusCamera.ERROR
            return

        self.status_connected_camera = StatusCamera.CONNECTED
        print("Поток камеры - старт")
        while True:
            ret, frame = cap.read()
            if ret:
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                frame_expanded = np.expand_dims(frame_rgb, axis=0)
                rgbImage = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = rgbImage.shape
                bytesPerLine = ch * w
                convertToQtFormat = QImage(
                    rgbImage.data, w, h, bytesPerLine, QImage.Format_RGB888
                )
                p = convertToQtFormat.scaled(700, 750, Qt.KeepAspectRatio)
                self.changePixmap.emit(p)

                if self.isFlagStop() == True:
                    self.status_connected_camera = StatusCamera.WAITING
                    print("Поток камеры - стоп")
                    #logger.info("Камера: Поток остановлен")
                    self.setFlagStop(False)
                    break
                try:
                    if self.isFlagSave():
                        cv2.imwrite(self.filename, frame)
                        self.resetSaveFoto()
                except Exception:
                    print("Неправильное имя файла!")
                    #logger.warning("Камера: Неправильное имя файла!")
                    self.resetSaveFoto()
            self.msleep(20)
        cv2.destroyAllWindows()

    def setFlagStop(self, bool_):
        with self.mutex_stop_camera:
            self.flag_stop_camera = bool_

    def isFlagStop(self):
        with self.mutex_stop_camera:
            return self.flag_stop_camera

    def isFlagSave(self):
        with self.mutex_save_foto:
            return self.flag_foto

    def resetSaveFoto(self):
        with self.mutex_save_foto:
            self.filename = ""
            self.flag_foto = False

    def setFoto(self, name):
        with self.mutex_save_foto:
            self.filename = name
            self.flag_foto = True