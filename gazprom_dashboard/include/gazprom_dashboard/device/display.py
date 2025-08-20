#!/usr/bin/python3
# -*- coding: utf-8 -*-



import time
from PyQt5.QtCore import  QObject, pyqtSignal
import threading
import datetime

from gazprom_msgs.msg import ManipulatorStatus as ManipulatorStatus
from gazprom_msgs.msg import OperatorMessage as OperatorMessage
from gazprom_msgs.msg import ParametrsPainting as ParametrsPainting
from gazprom_msgs.msg import StatusSystem as StatusSystem
# *********************************************
#    Поток обновления статусов на интерфейсе *
# Все статусы, информация и ошибки,          *
# здесь обновляется в отдельном цикле        *
# *********************************************

class BackendThreadUpdateDisplay(QObject):
    #  Defining signals through class member objects
    update_connect_camera = pyqtSignal(str)
    update_class = pyqtSignal(str)
    update_status_system = pyqtSignal(str)
    update_mode = pyqtSignal(str)
    update_koefficient_confident = pyqtSignal(str)
    update_marka = pyqtSignal(str)
    update_information = pyqtSignal(str)
    update_message_error_class = pyqtSignal(str)
    update_information_two_tab = pyqtSignal(str)
    update_message_error_class_tab_two = pyqtSignal(str)
    update_counter_all_parts = pyqtSignal(str)
    update_counter_current_parts = pyqtSignal(str)
    update_manipulator_status = pyqtSignal(str)
    update_time = pyqtSignal(str)

    _status_connected_camera = ""
    _status_foto = ""
    _status_system = ""
    _status_manipulator = ""

    status_sistem_app_ = False

    message = ""
    message_class_error = msg.manual_mode_message["wait"]

    flag_message = False
    flag_exec = False

    _marking = ""
    _quantity_per_batch = 0
    _party = 0

    information_two_tab = ""

    queue_display = ""
    counter_current_parts_data = 0

    mutex_status_system = threading.Lock()
    mutex_status_robot = threading.Lock()
    mutex_painting = threading.Lock()


    mutex_message = threading.Lock()
    mutex_counter = threading.Lock()
    mutex_queue = threading.Lock()

    mutex_flag_message = threading.Lock()
    mutex_flag_exec = threading.Lock()

    first_run_time = datetime.datetime(
        2025, 5, 21, 8, 0, 0
    )  # Пример: 19 марта 10:21:00





    def run(self):
        print("BackendThreadUpdateDisplay поток - старт")
        while True:
            current_time = datetime.datetime.now()
            working_time = current_time - self.first_run_time
            hours, remainder = divmod(working_time.seconds, 3600)
            minutes, seconds = divmod(remainder, 60)


            self.update_connect_camera.emit(
                self.getConnectStatus(self._status_connected_camera)
            )
            self.update_mode.emit(" Полуавтоматический")

            ###

            self.update_status_system.emit(self.getSystemtStatus(self._status_system))
            self.update_manipulator_status.emit(
                self.getManipulatorStatus(self._status_manipulator)
            )

            self.setParametrsPainting()

            


            self.update_message_error_class.emit(str(self.message_class_error))
            self.update_message_error_class_tab_two.emit(str(self.message_class_error))

           
            self.update_time.emit(
                str(working_time.days)
                + "d "
                + str(hours)
                + "h "
                + str(minutes)
                + "m "
                + str(seconds)
                + "s"
            )

            time.sleep(1)
            if self.isFlagExec():
                print("BackendThreadUpdateDisplay поток - стоп")
                #logger.info("BackendThreadUpdateDisplay: Поток остановлен")
                break
            if self.isFlagMessage():
                self.update_information.emit(str(self.message))
                self.setFlagMessage(False)

    def getConnectStatus(self, str_name):
        str_name = str(str_name)
        if str_name == "StatusCamera.NOT_LOADED":
            return "Не загружена"
        elif str_name == "StatusCamera.CONNECTED":
            return "Подключено"
        elif str_name == "StatusCamera.WAITING":
            return "Ожидание"
        else:
            return "Ошибка"

   


    def isFlagMessage(self):
        with self.mutex_flag_message:
            return self.flag_message

    def setFlagMessage(self, bool_):
        with self.mutex_flag_message:
            self.flag_message = bool_

    def isFlagExec(self):
        with self.mutex_flag_exec:
            return self.flag_exec

    def setFlagExec(self, bool_):
        with self.mutex_flag_exec:
            self.flag_exec = bool_

    # Обновляет статусы системы, камеры и манипулятора
    def updateStatus(self, connect, manipulator, system):
        with self.mutex_status:
            self._status_connected_camera = connect
            # self._status_system = system
            self._status_manipulator = manipulator

    # Марка, количество в партии и остаток
    def updateParametrsPainting(self, marking, quantity_per_batch, party):
        with self.mutex_painting:
            self._marking = marking
            self._quantity_per_batch = quantity_per_batch
            self._party = party

    def setCounterData(self, current):
        with self.mutex_counter:
            self.counter_current_parts_data = current

    def setQueueData(self, queue_detal_):
        with self.mutex_queue:
            self.queue_display = queue_detal_

    def setMessage(self, msg):
        with self.mutex_message:
            self.message = msg


    # **********************
    #    Статус  системы   *
    # **********************

    def updateStatusSystemCallback(self, msg: StatusSystem) -> None:
        with self.mutex_status_system:
            self._status_system = msg.state


    def getSystemtStatus(self):
        with self.mutex_status_system:
            state = self._status_system
            if state == StatusSystem.RUNNING:
                self.status_sistem_app_ = True
                return "Работает"
            elif state == StatusSystem.STOPPING:
                self.status_sistem_app_ = True
                return "Остановлена"
            elif state == StatusSystem.WAITING:
                self.status_sistem_app_ = False
                return "Ожидание"
            elif state == StatusSystem.OK:
                self.status_sistem_app_ = True
                return "Исправна"
            elif state == StatusSystem.ERROR:
                self.status_sistem_app_ = False
                return "Ошибка"
            else:
                #logger.error("Система: Неизвестное состояние")
                return "АААА"
       

    # **********************
    #    Статус  робота    *
    # **********************

    def updateManipulatorStatusCallback(self, msg: ManipulatorStatus) -> None:
        with self.mutex_status_robot:
            self._status_manipulator = msg.state

    def getManipulatorStatus(self):
        with self.mutex_status_robot:
            state = self._status_manipulator
            if state == ManipulatorStatus.RUNNING:
                return "Движение"
            elif state == ManipulatorStatus.STOPPING:
                return "Неподвижен"
            elif state == ManipulatorStatus.CONNECT:
                return "Подключено"
            elif state == ManipulatorStatus.ERROR_CONNECT:
                return "Не подключен"
            elif state == ManipulatorStatus.ERROR:
                return "Ошибка"
            elif state == ManipulatorStatus.TIMEOUT:
                return "Timeout"
            else:
                #logger.error("Робот: Неизвестное состояние")
                return "АААА"
            
    # *************************
    #    Параметры покраски   *
    # *************************

    def updateParametrsPaintingCallback(self, msg: ParametrsPainting) -> None:
        with self.mutex_painting:
            self._marking = msg.marking
            self._quantity_per_batch = msg.quantity_per_batch
            self._party = msg.party
            self.queue_display = msg.queue
            self.counter_current_parts_data = msg.counter_current_parts_data

    def setParametrsPainting(self):
        with self.mutex_painting:
            self.update_class.emit(str(self._party))
            self.update_koefficient_confident.emit(str(self._quantity_per_batch))
            self.update_marka.emit(str(self._marking))
            self.update_counter_all_parts.emit(str(self.queue_display))
            self.update_counter_current_parts.emit(str(self.counter_current_parts_data))


    # *************************
    #    Сообщение оператору  *
    # *************************

    def updateOperatorMessageCallback(self, msg: OperatorMessage) -> None:
        # CLTKFNM !!!
        ...