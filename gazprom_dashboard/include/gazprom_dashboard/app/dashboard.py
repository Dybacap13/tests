#!/usr/bin/python3
# -*- coding: utf-8 -*-


from PyQt5.QtWidgets import QMainWindow, QApplication, QFrame
import time
import sys
from PyQt5.QtWidgets import (
    QWidget,
    QTableWidget,
    QFormLayout,
    QTableWidgetItem,
    QTextEdit,
    QLabel,
    QDesktopWidget,
    QLineEdit,
    QTabWidget,
    QPushButton,
    QMessageBox,
    QComboBox,
    QDialog,
    QMainWindow,
    QPushButton,
    QVBoxLayout,
    QLabel,
    QFileDialog,
    QToolButton,
    QPlainTextEdit,
    QToolButton,
    QMenu,
    QWidgetAction,
)
from PyQt5.QtGui import (
    QColor,
    QFont,
    QImage,
    QPixmap,
    QIcon,
    QPainter,
    QPen,
    QBrush,
    QTextCharFormat,
    QColor,
    QTextCursor,
)
from PyQt5.QtCore import Qt, QObject, pyqtSignal, QThread, QTimer
import threading
from time import sleep
from pathlib import Path
import numpy as np
import cv2
import enum

import webbrowser
import os
# import msg.message_label_app as msg
import datetime

import csv
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# from dxf_parser import DXFViewer
# from ezdxf import recover
# from ezdxf.addons.drawing import matplotlib
import matplotlib.image as mpimg

from gazprom_msgs.msg import ManipulatorStatus as ManipulatorStatus
from gazprom_msgs.msg import OperatorMessage as OperatorMessage
from gazprom_msgs.msg import ParametrsPainting as ParametrsPainting
from gazprom_msgs.msg import StatusSystem as StatusSystem

import device.camera as camera
import device.display as display

import tools.trigger_service as trigger_service
from rclpy.node import Node
from example_interfaces.srv import Trigger

# *************************
#     Приложение         *
# *************************


class App(QMainWindow):

    def __init__(
        self,
        ###
        names_table,
        counter_table,
        error_table,
        id_table,
        ###
        ip_robot,
        port_robot,
        port_sensor,
        port_joint_states,
        port_joint_publisher,
        port_camera_,
        foto_delay_,
        socket_delay_robot_,
        socket_delay_joint_states_,
        socket_delay_publisher_,
        socket_delay_sensor_,
    ):
        super().__init__()

        # Расширенные настройки
        self.ip_socket = str(ip_robot)

        self.port_socket_robot = str(port_robot)
        self.port_socket_sensor = str(port_sensor)
        self.port_socket_joint_states = str(port_joint_states)
        self.port_socket_joint_publisher = str(port_joint_publisher)
        self.port_camera = str(port_camera_)

        self.socket_delay_robot = str(socket_delay_robot_)
        self.socket_delay_joint_states = str(socket_delay_joint_states_)
        self.socket_delay_publisher = str(socket_delay_publisher_)
        self.socket_delay_sensor = str(socket_delay_sensor_)

        self.delay_paint = str(foto_delay_)

        # Параметры для таблицы
        self.table_names = names_table
        self.table_error = error_table
        self.table_counter = counter_table
        self.table_id = id_table

        # Скорость конвейера и счетчики
        self.velocity_conveyor_data = 1.8
        self.queue_detal = " "
        self.counter_current_parts = 0

        self.camera_ = camera.ThreadOpenCV(self.port_camera)
        self.display_ = display.BackendThreadUpdateDisplay()

        # Шрифты
        self.round_button = QFont("Times", 10, QFont.Bold)
        self.oval_button = QFont("Times", 14, QFont.Bold)

        # Главное окно
        self.setWindowTitle("GHS")
        self.screen = QDesktopWidget().screenGeometry()
        self.setGeometry(0, 0, self.screen.width(), self.screen.height())
        self.setWindowIcon(QIcon("image/icon.png"))

        # Вкладки
        self.tab_widget = QTabWidget()
        self.setCentralWidget(self.tab_widget)
        self.tab_widget.setFont(self.oval_button)
        self.tab_widget.setStyleSheet(Path("gss/tab.qss").read_text())

        self.createTabOne()
        self.createTabTwo()
        # self.createTabThree()

        # Мютексы
        self.mutex = threading.Lock()
        self.mutex_reset = threading.Lock()
        self.mutex_behavior = threading.Lock()
        self.mutex_beckend = threading.Lock()
        self.mutex_setting = threading.Lock()
        self.mutex_flag_change_settings = threading.Lock()
        self.mutex_flag_change_timeout = threading.Lock()
        self.mutex_change_timeout = threading.Lock()
        self.mutex_port_camera = threading.Lock()
        self.mutex_delay_paint = threading.Lock()
        self.mutex_log = threading.Lock()

        # Флаги
        self.flag_start_camera = False
        self.flag_error = False
        self.flag_stop_programm = False
        self.flag_start_programm = False
        self.flag_change_settings = False
        self.flag_reset_error = False
        self.flag_change_port_camera = False
        self.flag_change_delay_paint = False
        self.flag_change_timeout = False

        # Гиперпараметры
        self.mark_manual = ""
        self.party_manual = 0

        # Параметры парсера
        self.scale_factor_x = 495 / 1100
        self.scale_factor_y = 395 / 1080
        self.start_point = None
        self.last_point = None
        self.points = []  # Список для точек
        self.line_plots = []  # Линии
        self.temp_line = None  # Временная линия
        self.cid_click = None  # Переменные для идентификаторов событий
        self.cid_move = None

        # Геометрия вкладок
        self.indentation_edges = 50
        self.offset_tab_two = 0

        # Вкладка 1
        self.drawLabelStatus()
        self.drawInputStates()
        self.drawButton()

        # Вкладка 2
        self.drawFieldTabTwo()
        self.createTableStatistics()
        self.drawButtonTabTwo()
        self.drawInputTabTwo()

        # # Вкладка 3
        # self.drawFieldTabThree()
        # self.drawButtonTabThree()
        # self.drawInputTabThree()

        # self.tab3.setLayout(self.layout_tab3)  # Устанавливаем макет вкладки

        self.setupUi()


        self.node = Node("dashboard_node")
        self.state_system_sub = self.node.create_subscription(StatusSystem, "behavior/status_system",self.display_.updateStatusSystemCallback, 10)
        self.state_robot_sub = self.node.create_subscription(ManipulatorStatus,"behavior/status_robot",self.display_.updateManipulatorStatusCallback, 10)
        self.paint_parametrs_sub = self.node.create_subscription(ParametrsPainting,"behavior/paint_parametrs",self.display_.updateParametrsPaintingCallback, 10)
        self.operator_message_sub = self.node.create_subscription(OperatorMessage,"behavior/operator_message",self.display_.updateOperatorMessageCallback, 10)



    # *************************
    #    Графическая часть   *
    # *************************

    def createTabOne(self):
        self.tab1 = QWidget()
        self.tab_widget.addTab(self.tab1, "Основное  окно")

    def createTabTwo(self):
        self.tab2 = QWidget()
        self.tab_widget.addTab(self.tab2, "Настройки   параметров")

    def createTabThree(self):
        self.tab3 = QWidget()
        self.layout_tab3 = QVBoxLayout(self.tab3)
        self.tab_widget.addTab(self.tab3, "DXF парсер")

    def drawFieldTabTwo(self):

        statistics_field = QLabel(self.tab2)  # rgba(0,0,0,0)  border: 1px solid black
        statistics_field.setGeometry(50, 50, 700, 715)
        statistics_field.setStyleSheet(
            "background-color: rgb(69,69,69) ;border-radius: 20px; "
        )

        statistics_label = QLabel("Статистика", self.tab2)
        statistics_label.setAlignment(Qt.AlignCenter)
        statistics_label.setGeometry(50, 57, 700, 50)
        statistics_label.setFont(self.oval_button)
        statistics_label.setStyleSheet(
            "background-color: rbg(0,0,0,0) ; color: white; "
        )

        self.message_error_class_field_tab_two = QLabel(" Сообщение: ", self.tab2)
        self.message_error_class_field_tab_two.setGeometry(
            50,
            665 + 20 + 70 - 30 + 70 + 20,
            700,
            55,
        )
        self.message_error_class_field_tab_two.setFont(self.oval_button)
        self.message_error_class_field_tab_two.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;   color: white"
        )

        socket_field = QLabel(self.tab2)  # rgba(0,0,0,0)  border: 1px solid black
        socket_field.setGeometry(
            self.screen.width() - self.indentation_edges - 1020, 50, 1020, 247 + 70
        )
        socket_field.setStyleSheet(
            "background-color: rgb(69,69,69) ;border-radius: 20px; "
        )
        socket_label = QLabel("Настройки ip адреса робота и портов общения", self.tab2)
        socket_label.setAlignment(Qt.AlignCenter)
        socket_label.setGeometry(
            self.screen.width() - self.indentation_edges - 1020, 57, 1020, 50
        )
        socket_label.setFont(self.oval_button)
        socket_label.setStyleSheet("background-color: rbg(0,0,0,0) ; color: white; ")

        camera_port_delay_field = QLabel(
            self.tab2
        )  # rgba(0,0,0,0)  border: 1px solid black
        camera_port_delay_field.setGeometry(
            self.screen.width() - self.indentation_edges - 1020,
            317 + 70,
            670,
            177 + 43 + 8,
        )
        camera_port_delay_field.setStyleSheet(
            "background-color: rgb(69,69,69) ;border-radius: 20px; "
        )

        camera_port_label = QLabel(" Порт камеры:", self.tab2)
        camera_port_label.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 50, 437 + 4, 300, 50
        )
        camera_port_label.setFont(self.oval_button)
        camera_port_label.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )

        self.camera_port_input = QLineEdit(self.tab2)
        self.camera_port_input.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 50 + 185,
            437 + 4,
            400,
            50,
        )
        self.camera_port_input.setFont(self.oval_button)
        self.camera_port_input.setStyleSheet(
            "background-color:rgba(0,0,0,0)  ;border-radius: 15px;  color: rgb(30, 144, 255);"
        )
        self.camera_port_input.setText(str(self.port_camera))

        delay_paint_label = QLabel(" Задержка :", self.tab2)
        delay_paint_label.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 50,
            437 + 4 + 50 + 20,
            300,
            50,
        )
        delay_paint_label.setFont(self.oval_button)
        delay_paint_label.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )

        self.delay_paint_input = QLineEdit(self.tab2)
        self.delay_paint_input.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 50 + 185,
            437 + 50 + 20 + 4,
            400,
            50,
        )
        self.delay_paint_input.setFont(self.oval_button)
        self.delay_paint_input.setStyleSheet(
            "background-color:rgba(0,0,0,0)  ;border-radius: 15px;  color: rgb(30, 144, 255);"
        )
        self.delay_paint_input.setText(str(self.delay_paint))

        ip_label = QLabel(" IP адрес робота:", self.tab2)
        ip_label.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 50 + 35, 127, 400, 50
        )
        ip_label.setFont(self.oval_button)
        ip_label.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )

        port_robot_label = QLabel(" Порт робота:", self.tab2)
        port_robot_label.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 50 + 85, 197, 300, 50
        )
        port_robot_label.setFont(self.oval_button)
        port_robot_label.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )

        port_sensor_label = QLabel(" Порт датчика:", self.tab2)
        port_sensor_label.setGeometry(
            self.screen.width()
            - self.indentation_edges
            - 1020
            + 50
            + 300
            + 20
            + 50
            + 160,
            197,
            300,
            50,
        )
        port_sensor_label.setFont(self.oval_button)
        port_sensor_label.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )

        port_robot_label = QLabel(" Порт паблишера:", self.tab2)
        port_robot_label.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 50 + 85,
            197 + 70,
            300,
            50,
        )
        port_robot_label.setFont(self.oval_button)
        port_robot_label.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )

        port_sensor_label = QLabel(" Порт джоинтов:", self.tab2)
        port_sensor_label.setGeometry(
            self.screen.width()
            - self.indentation_edges
            - 1020
            + 50
            + 300
            + 20
            + 50
            + 160,
            197 + 70,
            300,
            50,
        )
        port_sensor_label.setFont(self.oval_button)
        port_sensor_label.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )

        self.data_ip_input = QLineEdit(self.tab2)
        self.data_ip_input.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 50 + 35 + 50 + 180,
            127,
            400,
            50,
        )
        self.data_ip_input.setFont(self.oval_button)
        self.data_ip_input.setStyleSheet(
            "background-color:rgba(0,0,0,0)  ;border-radius: 15px;  color: rgb(30, 144, 255);"
        )
        self.data_ip_input.setText(self.ip_socket)

        self.data_port_robot_input = QLineEdit(self.tab2)
        self.data_port_robot_input.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 50 + 85 + 50 + 180,
            197,
            400,
            50,
        )
        self.data_port_robot_input.setFont(self.oval_button)
        self.data_port_robot_input.setStyleSheet(
            "background-color:rgba(0,0,0,0)  ;border-radius: 15px;  color: rgb(30, 144, 255);"
        )
        self.data_port_robot_input.setText(self.port_socket_robot)

        self.data_port_sensor_input = QLineEdit(self.tab2)
        self.data_port_sensor_input.setGeometry(
            self.screen.width()
            - self.indentation_edges
            - 1020
            + 50
            + +300
            + 20
            + 50
            + 160
            + 50
            + 180,
            197,
            400,
            50,
        )
        self.data_port_sensor_input.setFont(self.oval_button)
        self.data_port_sensor_input.setStyleSheet(
            "background-color:rgba(0,0,0,0)  ;border-radius: 15px;  color: rgb(30, 144, 255);"
        )
        self.data_port_sensor_input.setText(self.port_socket_sensor)

        self.data_port_publish_input = QLineEdit(self.tab2)
        self.data_port_publish_input.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 50 + 85 + 50 + 180,
            197 + 70,
            400,
            50,
        )
        self.data_port_publish_input.setFont(self.oval_button)
        self.data_port_publish_input.setStyleSheet(
            "background-color:rgba(0,0,0,0)  ;border-radius: 15px;  color: rgb(30, 144, 255);"
        )
        self.data_port_publish_input.setText(self.port_socket_joint_publisher)

        self.data_port_joint_input = QLineEdit(self.tab2)
        self.data_port_joint_input.setGeometry(
            self.screen.width()
            - self.indentation_edges
            - 1020
            + 50
            + +300
            + 20
            + 50
            + 160
            + 50
            + 180,
            197 + 70,
            400,
            50,
        )
        self.data_port_joint_input.setFont(self.oval_button)
        self.data_port_joint_input.setStyleSheet(
            "background-color:rgba(0,0,0,0)  ;border-radius: 15px;  color: rgb(30, 144, 255);"
        )
        self.data_port_joint_input.setText(self.port_socket_joint_states)

        password_label = QLabel(" Пароль для настройки:", self.tab2)
        password_label.setGeometry(
            self.screen.width() - self.indentation_edges - 1020,
            665 + 20 + 70 - 30 + 70 + 20,
            500,
            55,
        )

        password_label.setFont(self.oval_button)
        password_label.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )

        self.password_input = QLineEdit(self.tab2)
        self.password_input.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 250 + 60 - 15,
            665 + 20 + 70 - 30 + 70 + 20,
            380,
            55,
        )
        self.password_input.setFont(self.oval_button)
        self.password_input.setStyleSheet(
            "background-color:rgba(0,0,0,0)  ;border-radius: 15px;  color: rgb(30, 144, 255);"
        )
        self.password_input.setPlaceholderText("Введите пароль")

        self.info_settings_input = QTextEdit(self.tab2)
        self.info_settings_input.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 185, 635 + 45, 300, 70
        )
        self.info_settings_input.setFont(self.oval_button)
        self.info_settings_input.setStyleSheet(
            "background-color:rgba(0,0,0,0)  ;border-radius: 15px;  color: rgb(255, 255, 255);"
        )
        self.info_settings_input.setText(" Для настройки введите пароль")
        self.info_settings_input.setAlignment(Qt.AlignCenter)

        timeout_field = QLabel(self.tab2)  # rgba(0,0,0,0)  border: 1px solid black
        timeout_field.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 670 + 20,
            317 + 70,
            330,
            380 + 70 + 33,
        )
        timeout_field.setStyleSheet(
            "background-color: rgb(69,69,69) ;border-radius: 20px; "
        )

        timeout_label = QLabel("Настройки таймаутов", self.tab2)
        timeout_label.setAlignment(Qt.AlignCenter)
        timeout_label.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 670 + 20,
            317 + 70 + 15,
            330,
            50,
        )
        timeout_label.setFont(self.oval_button)
        timeout_label.setStyleSheet("background-color: rbg(0,0,0,0) ; color: white; ")

        timeout_label2 = QLabel("связи", self.tab2)
        timeout_label2.setAlignment(Qt.AlignCenter)
        timeout_label2.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 670 + 20,
            317 + 70 + 15 + 30,
            330,
            50,
        )
        timeout_label2.setFont(self.oval_button)
        timeout_label2.setStyleSheet("background-color: rbg(0,0,0,0) ; color: white; ")

        delay_robot_socket_label = QLabel(" Робот:", self.tab2)
        delay_robot_socket_label.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 670 + 20 + 40,
            317 + 70 + 70 + 33,
            250,
            50,
        )
        delay_robot_socket_label.setFont(self.oval_button)
        delay_robot_socket_label.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )

        delay_sensor_socket_label = QLabel(" Датчик:", self.tab2)
        delay_sensor_socket_label.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 670 + 20 + 40,
            317 + 70 * 3 + 33,
            250,
            50,
        )
        delay_sensor_socket_label.setFont(self.oval_button)
        delay_sensor_socket_label.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )

        delay_pub_socket_label = QLabel(" Паблишер:", self.tab2)
        delay_pub_socket_label.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 670 + 20 + 40,
            317 + 70 * 4 + 33,
            250,
            50,
        )
        delay_pub_socket_label.setFont(self.oval_button)
        delay_pub_socket_label.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )

        delay_joint_socket_label = QLabel(" Джоинты:", self.tab2)
        delay_joint_socket_label.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 670 + 20 + 40,
            317 + 70 * 5 + 33,
            250,
            50,
        )
        delay_joint_socket_label.setFont(self.oval_button)
        delay_joint_socket_label.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )

        self.delay_socket_input = QLineEdit(self.tab2)
        self.delay_socket_input.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 670 + 20 + 40 + 150,
            317 + 70 + 70 + 33,
            250,
            50,
        )
        self.delay_socket_input.setFont(self.oval_button)
        self.delay_socket_input.setStyleSheet(
            "background-color:rgba(0,0,0,0)  ;border-radius: 15px;  color: rgb(30, 144, 255);"
        )
        self.delay_socket_input.setText(str(self.socket_delay_robot))

        self.delay_sensor_input = QLineEdit(self.tab2)
        self.delay_sensor_input.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 670 + 20 + 40 + 150,
            317 + 70 * 3 + 33,
            250,
            50,
        )
        self.delay_sensor_input.setFont(self.oval_button)
        self.delay_sensor_input.setStyleSheet(
            "background-color:rgba(0,0,0,0)  ;border-radius: 15px;  color: rgb(30, 144, 255);"
        )
        self.delay_sensor_input.setText(str(self.socket_delay_sensor))

        self.delay_publisher_input = QLineEdit(self.tab2)
        self.delay_publisher_input.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 670 + 20 + 40 + 150,
            317 + 70 * 4 + 33,
            250,
            50,
        )
        self.delay_publisher_input.setFont(self.oval_button)
        self.delay_publisher_input.setStyleSheet(
            "background-color:rgba(0,0,0,0)  ;border-radius: 15px;  color: rgb(30, 144, 255);"
        )
        self.delay_publisher_input.setText(str(self.socket_delay_publisher))

        self.delay_joint_input = QLineEdit(self.tab2)
        self.delay_joint_input.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 670 + 20 + 40 + 150,
            317 + 70 * 5 + 33,
            250,
            50,
        )
        self.delay_joint_input.setFont(self.oval_button)
        self.delay_joint_input.setStyleSheet(
            "background-color:rgba(0,0,0,0)  ;border-radius: 15px;  color: rgb(30, 144, 255);"
        )
        self.delay_joint_input.setText(str(self.socket_delay_joint_states))

    def drawFieldTabThree(self):
        # Холст для отображения DXF
        self.fig = Figure(facecolor=(69 / 255, 69 / 255, 69 / 255))
        self.ax = self.fig.add_subplot(111)  # Добавляем оси

        self.ax.spines["bottom"].set_color((30 / 255, 144 / 255, 255 / 255))
        self.ax.spines["left"].set_color((30 / 255, 144 / 255, 255 / 255))

        self.ax.spines["bottom"].set_linewidth(2)
        self.ax.spines["left"].set_linewidth(2)

        self.ax.spines["top"].set_visible(False)
        self.ax.spines["right"].set_visible(False)

        self.ax.tick_params(
            axis="both", colors="white"
        )  # 'both' - применяется к обеим осям

        self.dxf_canvas = FigureCanvas(self.fig)
        self.dxf_canvas.setGeometry(700, 50, 300, 300)  # Размер и расположение

        self.layout_tab3.addWidget(self.dxf_canvas)  # Добавляем холст в макет
        self.tab3.setLayout(self.layout_tab3)  # Устанавливаем макет во вкладку

        self.coordinates_label = QLabel(" Координаты курсора: ", self.tab3)
        self.coordinates_label.setGeometry(250, 50, 470, 50)
        self.coordinates_label.setFont(self.oval_button)
        self.coordinates_label.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )

        z_label = QLabel(" Глубина: ", self.tab3)
        z_label.setGeometry(820, 50, 400, 50)
        z_label.setFont(self.oval_button)
        z_label.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )

        self.ax = self.fig.add_subplot(111)

    def drawInputTabTwo(self):
        self.message_error_class_input_tab_two = QLineEdit(self.tab2)
        self.message_error_class_input_tab_two.setGeometry(
            50 + 175,
            665 + 20 + 70 - 30 + 70 + 20,
            1020,
            55,
        )
        self.message_error_class_input_tab_two.setFont(self.oval_button)
        self.message_error_class_input_tab_two.setStyleSheet(
            "background-color: rgba(0,0,0,0) ;border-radius: 15px;   color: rgb(255, 0, 0)"
        )

    def drawInputTabThree(self):
        self.z_input = QLineEdit(self.tab3)
        self.z_input.setGeometry(950, 50, 250, 50)
        self.z_input.setPlaceholderText("Введите значение Z")
        self.z_input.setFont(self.oval_button)
        self.z_input.setStyleSheet(
            "background-color:rgba(0,0,0,0)  ;border-radius: 15px;  color: rgb(30, 144, 255);"
        )

        self.message_parser_dxf = QLineEdit(self.tab3)
        self.message_parser_dxf.setGeometry(950 + 250 + 100, 50, 410, 50)
        self.message_parser_dxf.setFont(self.oval_button)
        self.message_parser_dxf.setStyleSheet(
            "background-color:rgb(74,74,74)  ;border-radius: 15px;  color: white;"
        )

    def drawButtonTabTwo(self):

        self.reset_statistics = QPushButton("Сбросить", self.tab2)
        self.reset_statistics.setGeometry(70 + (150 + 20) * 2, 665, 150, 50)
        self.reset_statistics.setFont(self.oval_button)
        self.reset_statistics.setStyleSheet(Path("gss\system_button.qss").read_text())

        self.update_statistics = QPushButton("Обновить", self.tab2)
        self.update_statistics.setGeometry(70 + 150 + 20, 665, 150, 50)
        self.update_statistics.setFont(self.oval_button)
        self.update_statistics.setStyleSheet(Path("gss\system_button.qss").read_text())

        self.add_statistics = QPushButton("Добавить", self.tab2)
        self.add_statistics.setGeometry(70, 665, 150, 50)
        self.add_statistics.setFont(self.oval_button)
        self.add_statistics.setStyleSheet(Path("gss\system_button.qss").read_text())

        self.delete_statistics = QPushButton("Удалить", self.tab2)
        self.delete_statistics.setGeometry(70 + (150 + 20) * 3, 665, 150, 50)
        self.delete_statistics.setFont(self.oval_button)
        self.delete_statistics.setStyleSheet(Path("gss\system_button.qss").read_text())

        self.help_tab_two = QPushButton("Помощь", self.tab2)
        self.help_tab_two.setGeometry(
            self.screen.width() - self.indentation_edges - 90, 875, 100, 35
        )
        self.help_tab_two.setFont(self.round_button)
        self.help_tab_two.setStyleSheet(Path("gss\help.qss").read_text())

        self.change_socket_setting = QPushButton(
            "Установить текущие настройки", self.tab2
        )
        self.change_socket_setting.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 50 + 35 + 400 + 50,
            127,
            400,
            50,
        )
        self.change_socket_setting.setFont(self.oval_button)
        self.change_socket_setting.setStyleSheet(
            Path("gss\system_button.qss").read_text()
        )

        self.change_delay_paint = QPushButton("Изменить задержку", self.tab2)
        self.change_delay_paint.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 420 - 50,
            437 + 50 + 4 + 20,
            250,
            50,
        )
        self.change_delay_paint.setFont(self.oval_button)
        self.change_delay_paint.setStyleSheet(Path("gss\system_button.qss").read_text())

        self.change_delay_socket = QPushButton("Установить", self.tab2)
        self.change_delay_socket.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 670 + 20 + 40,
            317 + 70 * 6 + 33,
            250,
            50,
        )
        self.change_delay_socket.setFont(self.oval_button)
        self.change_delay_socket.setStyleSheet(
            Path("gss\system_button.qss").read_text()
        )

        self.change_port_camera = QPushButton("Изменить порт", self.tab2)
        self.change_port_camera.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 420 - 50,
            437 + 4,
            250,
            50,
        )
        self.change_port_camera.setFont(self.oval_button)
        self.change_port_camera.setStyleSheet(Path("gss\system_button.qss").read_text())

        self.password_button = QPushButton("Принять", self.tab2)
        self.password_button.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 520 + 20 - 20,
            665 + 20 + 70 - 30 + 70 + 20,
            150,
            55,
        )
        self.password_button.setFont(self.oval_button)
        self.password_button.setStyleSheet(Path("gss\system_button.qss").read_text())

        self.password_button.clicked.connect(self.enterPasswordSettings)

        self.change_socket_setting.clicked.connect(self.changeSocketSettings)

        self.change_port_camera.clicked.connect(self.changePortCamera)
        self.change_delay_paint.clicked.connect(self.changeDelayPaint)

        self.change_delay_socket.clicked.connect(self.changeTimeoutSocketSettings)

        self.change_socket_setting.setEnabled(False)
        self.change_port_camera.setEnabled(False)
        self.change_delay_paint.setEnabled(False)
        self.change_delay_socket.setEnabled(False)

        self.reset_statistics.clicked.connect(self.resetStatistics)
        self.update_statistics.clicked.connect(self.updateStatistics)
        self.help_tab_two.clicked.connect(self.help)
        self.add_statistics.clicked.connect(self.addRowinTable)
        self.delete_statistics.clicked.connect(self.deleteStatistics)

    def drawButtonTabThree(self):
        self.open_dxf = QPushButton(""" Открыть\n DXF файл """, self.tab3)
        self.open_dxf.setGeometry(20, 120, 150, 70)
        self.open_dxf.setFont(self.oval_button)
        self.open_dxf.setStyleSheet(Path("gss\system_button.qss").read_text())

        self.end_traj_dxf = QPushButton("Закончить\n траекторию", self.tab3)
        self.end_traj_dxf.setGeometry(20, 120 + (70 + 50) * 2, 150, 70)
        self.end_traj_dxf.setFont(self.oval_button)
        self.end_traj_dxf.setStyleSheet(Path("gss\system_button.qss").read_text())

        self.save_traj_dxf = QPushButton("Сохранить\n траекторию", self.tab3)
        self.save_traj_dxf.setGeometry(20, 120 + (70 + 50) * 3, 150, 70)
        self.save_traj_dxf.setFont(self.oval_button)
        self.save_traj_dxf.setStyleSheet(Path("gss\system_button.qss").read_text())

        self.send_traj_dxf = QPushButton("Отправить\nна робота", self.tab3)
        self.send_traj_dxf.setGeometry(20, 120 + (70 + 50) * 4, 150, 70)
        self.send_traj_dxf.setFont(self.oval_button)
        self.send_traj_dxf.setStyleSheet(Path("gss\system_button.qss").read_text())

        self.clear_traj_dxf = QPushButton("Отчистить\nтраекторию", self.tab3)
        self.clear_traj_dxf.setGeometry(20, 120 + (70 + 50) * 1, 150, 70)
        self.clear_traj_dxf.setFont(self.oval_button)
        self.clear_traj_dxf.setStyleSheet(Path("gss\system_button.qss").read_text())

        self.open_dxf.clicked.connect(self.openFileDXF)
        self.end_traj_dxf.clicked.connect(self.endTrajectoryDXF)
        self.save_traj_dxf.clicked.connect(self.saveTrajectoryDXF)
        self.send_traj_dxf.clicked.connect(self.sendTrajectoryDXF)
        self.clear_traj_dxf.clicked.connect(self.clearTrajectoryDXF)

    def createTableStatistics(self):

        self.table_widget = QTableWidget(self.tab2)
        self.table_widget.setRowCount(
            len(self.table_names)
        )  # Устанавливаем количество строк
        self.table_widget.setColumnCount(3)  # Устанавливаем количество столбцов
        self.table_widget.setGeometry(100, 100, 610, 515)

        self.table_widget.setFont(self.oval_button)
        self.table_widget.setHorizontalHeaderLabels(["Марка", "Количество", "Id"])
        self.table_widget.setStyleSheet(Path("gss/table.qss").read_text())
        self.table_widget.horizontalHeader().setFont(self.oval_button)
        self.table_widget.horizontalHeader().setStyleSheet(
            "background-color: rbg(0,0,0,0) ; color: rgb(30, 144, 255); "
        )
        self.table_widget.verticalHeader().setVisible(False)  # Скрываем индексы строк

        self.fillTableStatistics()

    def fillTableStatistics(self):

        data_in_table = []
        print("self.table_id")
        print(self.table_id)

        for i, element in enumerate(self.table_names):
            point = []
            point.append(self.table_names[i])
            point.append(self.table_counter[i])
            point.append(self.table_id[i])
            data_in_table.append(point)

        for row in range(len(data_in_table)):
            for column in range(len(data_in_table[row])):
                if column == 0:
                    column_lengh = 300
                else:
                    column_lengh = 150
                item = QTableWidgetItem(str(data_in_table[row][column]))
                item.setTextAlignment(Qt.AlignCenter)
                item.setForeground(QColor("white"))
                self.table_widget.setItem(row, column, item)
                self.table_widget.setRowHeight(row, 50)
                self.table_widget.setColumnWidth(column, column_lengh)

        self.table_widget.itemChanged.connect(self.tableChanged)

    def tableChanged(self, item):
        if (all(simbol in "1234567890" for simbol in item.text())) == False:
            return
        if item.text() == None or item.text() == "":
            return
        if item.column() == 1:
            self.table_counter[item.row()] = int(item.text())
        if item.column() == 2:
            self.table_id[item.row()] = int(item.text())

    def drawButton(self):
        self.start_camera = QPushButton("Start", self.tab1)
        self.start_camera.setGeometry(50 + 160, 660, 60, 60)
        self.start_camera.setFont(self.round_button)
        self.start_camera.setStyleSheet(Path("gss\start_save.qss").read_text())

        self.stop_camera = QPushButton("Stop", self.tab1)
        self.stop_camera.setGeometry(370, 660, 60, 60)
        self.stop_camera.setFont(self.round_button)
        self.stop_camera.setStyleSheet(Path("gss\stop.qss").read_text())

        self.save_image = QPushButton("Save", self.tab1)
        self.save_image.setGeometry(530, 660, 60, 60)
        self.save_image.setFont(self.round_button)
        self.save_image.setStyleSheet(Path("gss\start_save.qss").read_text())

        self.change_error_mark = QPushButton("Задать параметры покраски", self.tab1)
        self.change_error_mark.setGeometry(
            self.screen.width() - self.indentation_edges - 50 - 400, 705, 400, 55
        )
        self.change_error_mark.setFont(self.oval_button)
        self.change_error_mark.setStyleSheet(Path("gss\system_button.qss").read_text())

        self.launch_program = QPushButton("Запуск", self.tab1)
        self.launch_program.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 50 + 15, 137, 150, 55
        )
        self.launch_program.setFont(self.oval_button)
        self.launch_program.setStyleSheet(Path("gss\system_button.qss").read_text())

        self.stop_program = QPushButton("Остановка", self.tab1)
        self.stop_program.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 50 + 235, 137, 150, 55
        )
        self.stop_program.setFont(self.oval_button)
        self.stop_program.setStyleSheet(Path("gss\ostanovka.qss").read_text())

        self.help_tab_one = QPushButton("Помощь", self.tab1)
        self.help_tab_one.setGeometry(
            self.screen.width() - self.indentation_edges - 90, 875, 100, 35
        )
        self.help_tab_one.setFont(self.round_button)
        self.help_tab_one.setStyleSheet(Path("gss\help.qss").read_text())

        self.reset_error = QPushButton("Сброс ошибки", self.tab1)
        self.reset_error.setGeometry(
            self.screen.width() - self.indentation_edges - 50 - 400, 350, 400 - 120, 55
        )
        # self.reset_error.setGeometry(
        #     self.screen.width() - self.indentation_edges - 90  - 20 - 150, 875, 150, 35
        # )
        self.reset_error.setFont(self.oval_button)
        self.reset_error.setStyleSheet(Path("gss/reset_error.qss").read_text())

        self.log_button = QToolButton(self.tab1)
        self.log_button.setText("Журнал")
        self.log_button.setPopupMode(QToolButton.InstantPopup)
        self.log_button.setGeometry(
            self.screen.width() - self.indentation_edges - 120, 824, 100, 30
        )
        self.log_button.setFont(self.round_button)
        self.log_button.setStyleSheet(
            "background-color: rgb(74,74,74) ; color: white; "
        )

        self.log_menu = QMenu(self)

        # Виджет для отображения логов (можно заменить на QTextEdit)
        # self.log_plain_text = QPlainTextEdit(self.tab1)
        self.log_plain_text = ColoredLogger()
        self.log_plain_text.setReadOnly(True)
        self.log_plain_text.setFixedSize(1050, 500)
        self.log_plain_text.setFont(self.oval_button)
        self.log_plain_text.setStyleSheet(
            "background-color: rgb(74,74,74) ; color: white; "
        )

        # Добавляем виджет в меню
        log_widget_action = QWidgetAction(self)
        log_widget_action.setDefaultWidget(self.log_plain_text)
        self.log_menu.addAction(log_widget_action)

        # Устанавливаем меню для кнопки
        self.log_button.setMenu(self.log_menu)

        self.start_camera.clicked.connect(self.startCameraWork)
        self.stop_camera.clicked.connect(self.stopCameraWork)
        self.save_image.clicked.connect(self.saveFoto)
        self.stop_program.clicked.connect(self.stopProgramm)
        self.change_error_mark.clicked.connect(self.selectMarking)
        self.launch_program.clicked.connect(self.launchProgramm)
        self.help_tab_one.clicked.connect(self.help)
        self.reset_error.clicked.connect(self.resetError)

    def drawInputStates(self):

        self.input_status_connect = QLineEdit(self.tab1)
        self.input_status_connect.setGeometry(453, 120, 500, 50)
        self.input_status_connect.setFont(self.oval_button)
        self.input_status_connect.setStyleSheet(
            "background-color:rgba(0,0,0,0)  ;border-radius: 15px;  color: rgb(30, 144, 255);"
        )

        self.manipulator_status_input = QLineEdit(self.tab1)
        self.manipulator_status_input.setGeometry(453, 50, 500, 50)
        self.manipulator_status_input.setFont(self.oval_button)
        self.manipulator_status_input.setStyleSheet(
            "background-color:rgba(0,0,0,0)  ;border-radius: 15px;  color: rgb(30, 144, 255);"
        )

        self.status_system_input = QLineEdit(self.tab1)
        self.status_system_input.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 50 + 235, 350, 400, 55
        )
        self.status_system_input.setFont(self.oval_button)
        self.status_system_input.setStyleSheet(
            "background-color:rgba(0,0,0,0)  ;border-radius: 15px;  color: rgb(30, 144, 255);"
        )

        self.mode_system_input = QLineEdit(self.tab1)
        # self.mode_system_input.setGeometry(1270, 130, 400, 70)
        self.mode_system_input.setGeometry(
            self.screen.width() - self.indentation_edges - 400 - 50, 130, 400 - 120, 70
        )
        # self.screen.width()
        self.mode_system_input.setFont(self.oval_button)
        self.mode_system_input.setAlignment(Qt.AlignCenter)
        self.mode_system_input.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;   color: white"
        )

        self.status_classificator_input = QLineEdit(
            self.tab1
        )  # rgba(0,0,0,0)  border: 1px solid black
        self.status_classificator_input.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 50 + 235, 635, 400, 55
        )
        self.status_classificator_input.setFont(self.oval_button)
        self.status_classificator_input.setStyleSheet(
            "background-color:rgba(0,0,0,0)  ;border-radius: 15px;  color: rgb(30, 144, 255);"
        )

        self.marka_input = QLineEdit(self.tab1)
        self.marka_input.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 50 + 235 - 125 - 10,
            565,
            400,
            55,
        )
        self.marka_input.setFont(self.oval_button)
        self.marka_input.setStyleSheet(
            "background-color:rgba(0,0,0,0)  ;border-radius: 15px;  color: rgb(30, 144, 255);"
        )

        self.koefficient_confident_input = QLineEdit(self.tab1)
        self.koefficient_confident_input.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 50 + 235, 710, 400, 55
        )
        self.koefficient_confident_input.setFont(self.oval_button)
        self.koefficient_confident_input.setStyleSheet(
            "background-color:rgba(0,0,0,0)  ;border-radius: 15px;  color: rgb(30, 144, 255);"
        )

        self.message_error_class_input = QLineEdit(self.tab1)
        self.message_error_class_input.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 175, 810, 1020, 55
        )
        self.message_error_class_input.setFont(self.oval_button)
        self.message_error_class_input.setStyleSheet(
            "background-color: rgba(0,0,0,0) ;border-radius: 15px;   color: rgb(255, 0, 0)"
        )

        self.information_input = QLineEdit(self.tab1)
        self.information_input.setGeometry(225, 760, 1020, 70)
        self.information_input.setFont(self.oval_button)
        self.information_input.setStyleSheet(
            "background-color: rgba(0,0,0,0) ;border-radius: 15px;   color: rgb(30, 144, 255)"
        )

        self.output_queue_input = QLineEdit(self.tab1)
        self.output_queue_input.setGeometry(
            self.screen.width() - self.indentation_edges - 400 - 50, 635, 400, 55
        )

        self.output_queue_input.setFont(self.oval_button)
        self.output_queue_input.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )

        self.counter_current_parts_input = QLineEdit(self.tab1)
        self.counter_current_parts_input.setGeometry(
            self.screen.width() - self.indentation_edges - 400 - 50 + 300, 565, 400, 55
        )
        self.counter_current_parts_input.setFont(self.oval_button)
        self.counter_current_parts_input.setStyleSheet(
            "background-color: rgba(0,0,0,0) ;border-radius: 15px;   color: rgb(30, 144, 255)"
        )

        self.time_input = QLineEdit(self.tab1)
        self.time_input.setGeometry(50, 760 + 70 + 20, 600, 50)
        self.time_input.setFont(self.oval_button)
        self.time_input.setStyleSheet(
            "background-color: rgba(0,0,0,0) ;border-radius: 15px;   color: rgb(30, 144, 255)"
        )

    def drawLabelStatus(self):

        logo = QLabel(self.tab1)
        try:
            pixmap = QPixmap("image\logo_.png")
        except:
            logger.exception("Приложение: исключении при загрузки иконки")
        logo.setPixmap(pixmap)
        logo.setGeometry(20, 15, 100, 50)

        information_field = QLabel(" Информация: ", self.tab1)
        information_field.setGeometry(50, 760, 700, 70)
        information_field.setFont(self.oval_button)
        information_field.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;   color: white"
        )

        system_status_field = QLabel(
            self.tab1
        )  # rgba(0,0,0,0)  border: 1px solid black
        system_status_field.setGeometry(
            self.screen.width() - self.indentation_edges - 1020, 50 + 20 + 200, 500, 200
        )
        system_status_field.setStyleSheet(
            "background-color: rgb(69,69,69) ;border-radius: 20px; "
        )

        mode_field = QLabel(self.tab1)  # rgba(0,0,0,0)  border: 1px solid black
        # mode_field.setGeometry(1220, 50, 500, 200)
        mode_field.setGeometry(
            (self.screen.width() - self.indentation_edges - 500), 50, 500 - 120, 200
        )
        mode_field.setStyleSheet(
            "background-color: rgb(69,69,69) ;border-radius: 20px; "
        )

        counter_field = QLabel(self.tab1)  # rgba(0,0,0,0)  border: 1px solid black
        counter_field.setGeometry(
            (self.screen.width() - self.indentation_edges - 500), 270, 500 - 120, 200
        )
        counter_field.setStyleSheet(
            "background-color: rgb(69,69,69) ;border-radius: 20px; "
        )

        class_status_field = QLabel(self.tab1)  # rgba(0,0,0,0)  border: 1px solid black
        class_status_field.setGeometry(
            self.screen.width() - self.indentation_edges - 1020,
            270 + 20 + 200,
            1020,
            300,
        )
        class_status_field.setStyleSheet(
            "background-color: rgb(69,69,69) ;border-radius: 20px; "
        )

        manipulator_field = QLabel(self.tab1)  # rgba(0,0,0,0)  border: 1px solid black
        manipulator_field.setGeometry(
            self.screen.width() - self.indentation_edges - 1020, 50, 500, 200
        )
        manipulator_field.setStyleSheet(
            "background-color: rgb(69,69,69) ;border-radius: 20px; "
        )

        manipulator_button_field = QLabel(
            self.tab1
        )  # rgba(0,0,0,0)  border: 1px solid black
        manipulator_button_field.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 50, 130, 400, 70
        )
        manipulator_button_field.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 20px; "
        )

        system_status_label = QLabel("Состояние системы", self.tab1)
        system_status_label.setAlignment(Qt.AlignCenter)
        system_status_label.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 125, 275, 250, 50
        )
        system_status_label.setFont(self.oval_button)
        system_status_label.setStyleSheet(
            "background-color: rbg(0,0,0,0) ; color: white; "
        )

        mode_label = QLabel("Режим управления", self.tab1)
        mode_label.setAlignment(Qt.AlignCenter)
        # mode_label.setGeometry((1345), 55, 250, 50)
        mode_label.setGeometry(
            (self.screen.width() - self.indentation_edges - 500), 55, 500 - 120, 50
        )
        # self.setGeometry(0, 0, self.screen.width(), self.screen.height())

        mode_label.setFont(self.oval_button)
        mode_label.setStyleSheet("background-color: rbg(255,0,0,0) ; color: white; ")

        classification_label = QLabel("Информация о покраске", self.tab1)
        classification_label.setAlignment(Qt.AlignCenter)
        classification_label.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 310, 495, 400, 50
        )
        classification_label.setFont(self.oval_button)
        classification_label.setStyleSheet(
            "background-color: rbg(0,0,0,0) ; color: white; "
        )

        camera_label = QLabel(" Основная камера: ", self.tab1)
        camera_label.setGeometry(150, 120, 500, 50)
        camera_label.setFont(self.oval_button)
        camera_label.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )
        manipulator_status_label = QLabel(" Манипулятор: ", self.tab1)
        manipulator_status_label.setGeometry(150, 50, 500, 50)
        manipulator_status_label.setFont(self.oval_button)
        manipulator_status_label.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )

        system_label = QLabel(" Статус: ", self.tab1)
        system_label.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 50, 350, 400, 55
        )
        system_label.setFont(self.oval_button)
        system_label.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )

        counter_current_label = QLabel(" Количество за день: ", self.tab1)
        counter_current_label.setGeometry(
            self.screen.width() - self.indentation_edges - 400 - 50, 565, 400, 55
        )
        counter_current_label.setFont(self.oval_button)
        counter_current_label.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )

        classificator_label = QLabel(" Марка: ", self.tab1)
        classificator_label.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 50, 565, 400, 55
        )
        classificator_label.setFont(self.oval_button)
        classificator_label.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )

        marka_label = QLabel(" Кол-во в партии: ", self.tab1)
        marka_label.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 50, 635, 400, 55
        )
        marka_label.setFont(self.oval_button)
        marka_label.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )

        confident_label = QLabel(" Остаток партии: ", self.tab1)
        confident_label.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 50, 710, 400, 55
        )
        confident_label.setFont(self.oval_button)
        confident_label.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )

        manipulator_label = QLabel("Манипулятор", self.tab1)
        manipulator_label.setAlignment(Qt.AlignCenter)
        manipulator_label.setGeometry(
            self.screen.width() - self.indentation_edges - 1020 + 125, 55, 250, 50
        )
        manipulator_label.setFont(self.oval_button)
        manipulator_label.setStyleSheet(
            "background-color: rbg(0,0,0,0) ; color: white; "
        )

        self.message_error_class_field = QLabel(" Сообщение: ", self.tab1)
        self.message_error_class_field.setGeometry(
            self.screen.width() - self.indentation_edges - 1020, 810, 1020, 55
        )
        self.message_error_class_field.setFont(self.oval_button)
        self.message_error_class_field.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;   color: white"
        )

        self.label_video = QLabel(self.tab1)
        self.label_video.setGeometry(50, 190, 700, 450)
        self.label_video.setStyleSheet(
            "background-color: rgb(69,69,69) ;border-radius: 20px; "
        )

        self.signal_red = Circle(color_=QColor(69, 69, 69), parent=self.tab1)
        self.signal_red.setGeometry(
            (self.screen.width() - self.indentation_edges - 50 - 35), 50 + 125, 50, 50
        )

        self.signal_yellow = Circle(color_=QColor(69, 69, 69), parent=self.tab1)
        self.signal_yellow.setGeometry(
            (self.screen.width() - self.indentation_edges - 50 - 35),
            50 + 50 + 10 + 125,
            50,
            50,
        )

        self.signal_green = Circle(color_=QColor(69, 69, 69), parent=self.tab1)
        self.signal_green.setGeometry(
            (self.screen.width() - self.indentation_edges - 50 - 35),
            50 + 50 + 10 + 50 + 10 + 125,
            50,
            50,
        )




    def setupUi(self):

        self.setWindowTitle("GHS")
        try:
            self.setWindowIcon(QIcon("image/icon.png"))
        except:
            logger.exception("Приложение: исключение при загрузки иконки 2")

        self.setStyleSheet("background-color:  rgb(64,64,64);")


        self.timer_reset_error = QTimer()
        self.timer_reset_error.timeout.connect(self.blinkinButtonResetError)
         #  Connection signal
        self.display_.update_connect_camera.connect(self.handleDisplayConnect)
        self.display_.update_class.connect(self.handleDisplayClass)
        self.display_.update_status_system.connect(self.handleDisplayName)
        self.display_.update_mode.connect(self.handleDisplaySystem)
        self.display_.update_koefficient_confident.connect(
            self.handleDisplayKoefficientConfidens
        )
        self.display_.update_marka.connect(self.handleDisplayMarka)
        self.display_.update_information.connect(self.handleDisplayInformation)
        self.display_.update_message_error_class.connect(self.handleMessageErrorClass)
        self.display_.update_message_error_class_tab_two.connect(
            self.handleMessageErrorClassTabTwo
        )
        self.display_.update_counter_all_parts.connect(self.handleCounterAllParts)
        self.display_.update_counter_current_parts.connect(
            self.handleCounterCurrentParts
        )
        self.display_.update_manipulator_status.connect(self.handleManipulatorStatus)
        self.display_.update_time.connect(self.handleTime)

        self.thread = QThread()
        self.display_.moveToThread(self.thread)
        #  Start thread
        self.thread.started.connect(self.display_.run)
        self.thread.start()
        self.display_.setQueueData(self.queue_detal) # TODO
        self.show()

    # *********************************************
    #    Чтобы обновлять статусы в интерфейсе     *
    # *********************************************
    def handleTime(self, data):
        self.time_input.setText(data)

    def handleManipulatorStatus(self, data):
        self.manipulator_status_input.setText(data)

    def handleCounterAllParts(self, data):
        self.output_queue_input.setText(data)

    def handleCounterCurrentParts(self, data):
        self.counter_current_parts_input.setText(data)

    def handleDisplayConnect(self, data):
        self.input_status_connect.setText(data)

    def handleDisplayClass(self, data):
        self.status_classificator_input.setText(data)

    def handleDisplayName(self, data):
        self.status_system_input.setText(data)

    def handleDisplaySystem(self, data):
        self.mode_system_input.setText(data)

    def handleDisplayKoefficientConfidens(self, data):
        self.koefficient_confident_input.setText(data)

    def handleDisplayMarka(self, data):
        self.marka_input.setText(data)

    def handleDisplayInformation(self, data):
        # self.input_information.insertPlainText(data)
        self.information_input.setText(data)

    def handleMessageErrorClass(self, data):
        self.message_error_class_input.setText(data)

    def handleMessageErrorClassTabTwo(self, data):
        self.message_error_class_input_tab_two.setText(data)

    def updateStatusApp(
        self,
        connect,
        system,
        manipulator,
        message_,
        marking,
        remainder_of_party,
        all_party,
    ):
        self.display_.updateStatus(connect, manipulator, system)
        self.display_.updateParametrsPainting(marking, remainder_of_party, all_party)
        self.display_.setMessage(message_)

    # ******************
    #    С камерой    *
    # ******************

    def startCameraWork(self):
        self.thread_videa.changePixmap.connect(self.setImageCamera)
        self.thread_videa.start()
        self.flag_start_camera = True

    def setImageCamera(self, image):
        self.label_video.setPixmap(QPixmap.fromImage(image))

    def stopCameraWork(self):
        if self.flag_start_camera == True:
            self.thread_videa.setFlagStop(True)
            self.flag_start_camera = False

    # ******************
    #    Выключение   *
    # ******************

    def closeEvent(self, event):
        reply = QMessageBox.question(
            self,
            "Message",
            "Are you sure to quit?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            self.shuttingAllThread()
        else:
            event.ignore()

    def shuttingAllThread(self):
        if self.flag_start_camera == True:
            self.thread_videa.setFlagStop(True)
        self.display_.setFlagExec(True)

    # ******************
    #    Кнопочки     *
    # ******************

    def selectMarking(self):
        dialog = QDialog()
        dialog.setWindowTitle("Выбор марки")
        dialog.setGeometry(900, 500, 400, 290)
        dialog.setStyleSheet(
            "background-color: rgb(69, 69, 69); border-radius: 15px; color: white;"
        )
        layout = QVBoxLayout()

        # Информация
        label_info = QLabel("Выберите марку из списка:", self)
        label_info.setGeometry(50, 50, 300, 50)
        label_info.setFont(self.oval_button)
        label_info.setStyleSheet(
            "QComboBox {background-color: rgb(74,74,74);  color: white; border-radius: 20px;}"
        )
        layout.addWidget(label_info)

        # Список
        combo = QComboBox()
        combo.setGeometry(50, 120, 300, 50)
        combo.setFont(self.oval_button)
        combo.setStyleSheet(
            "QComboBox {background-color: rgb(74,74,74);  color: rgb(30, 144, 255); border-radius: 20px;}"
        )
        markers_list = self.table_names.copy()
        markers_list.insert(0, "Не выбрано")
        combo.addItems(markers_list)
        layout.addWidget(combo)

        label_info2 = QLabel("Количество деталей в партии:", self)
        label_info2.setGeometry(50, 50, 300, 50)
        label_info2.setFont(self.oval_button)
        label_info2.setStyleSheet(
            "QComboBox {background-color: rgb(74,74,74);  color: white; border-radius: 20px;}"
        )
        layout.addWidget(label_info2)

        counter_det = QLineEdit()
        counter_det.setGeometry(150, 190, 300, 50)
        counter_det.setStyleSheet(
            (
                "background-color: rgb(74,74,74) ;border-radius: 15px;  color:  rgb(30, 144, 255)"
            )
        )
        counter_det.setFont(self.oval_button)
        counter_det.setPlaceholderText("Укажите количество")
        layout.addWidget(counter_det)

        button = QPushButton("OK")
        button.setGeometry(175, 190, 150, 50)
        button.setFont(self.oval_button)
        button.setStyleSheet(Path("gss\system_button.qss").read_text())
        button.clicked.connect(dialog.accept)
        layout.addWidget(button)
        dialog.setLayout(layout)

        if dialog.exec_() == QDialog.Accepted:
            selected_mark = combo.currentText()
            if selected_mark == "Не выбрано":
                with self.mutex:
                    self.display_.message_class_error = msg.error_message["mark_none"]
                    logger.warning(
                        "Приложение: марка не выбрана"
                    )  # "Марка не выбрана. Попробуйте ещё раз!"
                return
            try:
                counter_int = int(counter_det.text())
            except ValueError:
                with self.mutex:
                    self.display_.message_class_error = msg.error_message[
                        "number_detal_none"
                    ]
                    logger.warning(
                        "Приложение: Количество деталей должно быть натуральным числом!"
                    )
                return

            self.mark_manual = selected_mark
            self.party_manual = counter_int
            with self.mutex:
                self.display_.message_class_error = (
                    selected_mark + " в количестве " + counter_det.text() + " шт."
                )
        else:
            with self.mutex:
                self.display_.message_class_error = msg.error_message["mark_none"]
                logger.warning(
                    "Приложение: Марка не выбрана"
                )  # "Марка не выбрана. Попробуйте ещё раз!"

    def saveFoto(self):
        if self.flag_start_camera == False:
            with self.mutex:
                self.display_.message_class_error = msg.error_message[
                    "camera_off"
                ]  # "Для сохранения фотографии нужно включить камеру"
            return
        dialog = QDialog()
        dialog.setWindowTitle("Сохранить фотографию")
        dialog.setGeometry(200, 200, 1500, 100)
        dialog.setFont(self.oval_button)
        dialog.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )
        layout = QFormLayout()
        path_imput = QLineEdit()
        path_imput.setGeometry(300, 50, 150, 20)
        path_imput.setStyleSheet(
            (
                "background-color: rgb(69,69,69) ;border-radius: 15px;  color:  rgb(30, 144, 255)"
            )
        )
        path_imput.setFont(self.oval_button)
        layout.addRow(
            "Укажите путь до файла (например: D:/directory/file.png):", path_imput
        )
        submit_button = QPushButton("OK")
        submit_button.setStyleSheet(Path("gss\system_button.qss").read_text())
        submit_button.setFont(self.oval_button)

        def accept_dialog():
            path = path_imput.text()
            if path == "":
                with self.mutex:
                    self.display_.message_class_error = msg.error_message["path_none"]
                    logger.warning(
                        "Приложение: Путь не может быть пустым!"
                    )  # "Путь не может быть пустым!"
            else:
                self.thread_videa.setFilename(path)
                self.thread_videa.setFlagSave(True)
                # self.thread_videa.filename = path
                # self.thread_videa.flag_foto = True
            dialog.accept()

        submit_button.clicked.connect(accept_dialog)
        layout.addWidget(submit_button)
        dialog.setLayout(layout)
        dialog.exec_()


    # *****************************************
    #    Работа со статистикой - таблицей     *
    # *****************************************

    def resetStatistics(self):
        for i in range(0, len(self.table_counter)):
            self.table_counter[i] = 0
        self.fillTableStatistics()

    def updateStatistics(self):
        self.fillTableStatistics()

    def deleteStatistics(self):
        self.getDeleteNameMarkTable()

    def addRowinTable(self):
        self.getNewNameMarkTable()
        print()
        if len(self.table_names) == self.table_widget.rowCount():
            return
        self.table_counter.append(0)
        row_count = self.table_widget.rowCount()
        self.table_widget.insertRow(row_count)
        self.table_widget.setItem(row_count, 0, QTableWidgetItem(self.table_names[-1]))
        self.table_widget.setItem(
            row_count, 1, QTableWidgetItem(str(self.table_counter[-1]))
        )

        self.table_widget.setItem(
            row_count, 2, QTableWidgetItem(str(self.table_id[-1]))
        )

    def getNewNameMarkTable(self):
        dialog = QDialog()
        dialog.setWindowTitle("Введите новую марку детали и id")
        dialog.setGeometry(700, 500, 500, 100)
        dialog.setFont(self.oval_button)
        dialog.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )
        layout = QFormLayout()
        mark_input = QLineEdit()
        mark_input.setGeometry(300, 50, 150, 20)
        mark_input.setStyleSheet(
            (
                "background-color: rgb(69,69,69) ;border-radius: 15px;  color:  rgb(30, 144, 255)"
            )
        )

        id_input = QLineEdit()
        id_input.setGeometry(300, 50, 150, 20)
        id_input.setStyleSheet(
            (
                "background-color: rgb(69,69,69) ;border-radius: 15px;  color:  rgb(30, 144, 255)"
            )
        )
        mark_input.setFont(self.oval_button)
        id_input.setFont(self.oval_button)
        layout.addRow("Марка:", mark_input)
        layout.addRow("ID:", id_input)
        submit_button = QPushButton("OK")
        submit_button.setStyleSheet(Path("gss\system_button.qss").read_text())
        submit_button.setFont(self.oval_button)

        def accept_dialog():
            mark = mark_input.text()
            id = id_input.text()
            if mark == "":
                with self.mutex:
                    self.display_.message_class_error = msg.error_message["mark_none"]

            else:
                print("fsfsf")
                self.table_names.append(mark)
            try:
                self.table_id.append(int(id))
                print(self.table_id[-1])
            except:
                with self.mutex:
                    self.display_.message_class_error = "ID должно быть целым числом!"

            dialog.accept()

        submit_button.clicked.connect(accept_dialog)
        layout.addWidget(submit_button)
        dialog.setLayout(layout)
        dialog.exec_()

    def getDeleteNameMarkTable(self):
        dialog = QDialog()
        dialog.setWindowTitle("Введите  марку детали для удаления")
        dialog.setGeometry(700, 500, 500, 100)
        dialog.setFont(self.oval_button)
        dialog.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )
        layout = QFormLayout()
        mark_input = QLineEdit()
        mark_input.setGeometry(300, 50, 150, 20)
        mark_input.setStyleSheet(
            (
                "background-color: rgb(69,69,69) ;border-radius: 15px;  color:  rgb(30, 144, 255)"
            )
        )
        mark_input.setFont(self.oval_button)
        layout.addRow("Марка:", mark_input)
        submit_button = QPushButton("OK")
        submit_button.setStyleSheet(Path("gss\system_button.qss").read_text())
        submit_button.setFont(self.oval_button)

        def accept_dialog():
            mark = mark_input.text()
            if mark == "":
                with self.mutex:
                    self.display_.message_class_error = msg.error_message["mark_none"]
                    logger.warning("Приложение: Марка не задана в таблице")
            else:
                print("fsfsf")
                try:
                    self.table_names.remove(mark)
                except ValueError:
                    logger.warning(
                        "Приложение: Марка для удаления не найдена в таблице"
                    )

            dialog.accept()

        submit_button.clicked.connect(accept_dialog)
        layout.addWidget(submit_button)
        dialog.setLayout(layout)
        dialog.exec_()

    # *********************
    #    Стоп - Старт     *
    # *********************
    def stopProgramm(self):
        print("STOP")
        self.flag_stop_programm = True
        msg_box = QMessageBox(self)
        msg_box.setWindowTitle("Остановка программы")
        msg_box.setGeometry(800, 500, 200, 200)
        msg_box.setFont(self.oval_button)
        msg_box.setStyleSheet(
            "background-color: rgb(74,74,74) ;border-radius: 15px;  color: white"
        )
        msg_box.setText(
            "<center>Необходимо перезапустить программу с пульта робота</center>"
        )
        msg_box.exec_()

    def launchProgramm(self):
        with self.mutex:
            if self.display_.status_sistem_app_ == False:
                return
        self.flag_start_programm = True
        with self.mutex:
            self.display_.message_class_error = msg.manual_mode_message["wait"]

    def help(self):
        try:
            webbrowser.open_new_tab(self.resourcePath("./html/help.html"))
        except FileNotFoundError:
            logger.exception("Приложение: Файл инструкции не найден")
            return

    # Если нужен абсолютный путь
    def resourcePath(self, relative_path):
        try:
            base_path = sys.MEIPASS
        except Exception:
            base_path = os.path.abspath(".")
        return os.path.join(base_path, relative_path)

    # ***************************
    #    Расширенные настройки  *
    # ***************************

    def enterPasswordSettings(self):
        if self.password_input.text() == "654321":

            self.change_socket_setting.setEnabled(True)
            self.change_port_camera.setEnabled(True)
            self.change_delay_paint.setEnabled(True)
            self.change_delay_socket.setEnabled(True)
            self.info_settings_input.setText(
                "Пароль верный. После изменений перезагрузите приложение."
            )
        else:
            self.info_settings_input.setText("Пароль неправильный.")
            logger.warning("Приложение: Пароль неправильный")

    def setFlagChangeSettings(self, bool_):
        with self.mutex_flag_change_settings:
            self.flag_change_settings = bool_

    def getFlagChangeSettings(self):
        with self.mutex_flag_change_settings:
            return self.flag_change_settings

    def getIpSocket(self):
        with self.mutex_setting:
            return self.ip_socket

    def getPortSocketRobot(self):
        with self.mutex_setting:
            return self.port_socket_robot

    def getPortSocketSensor(self):
        with self.mutex_setting:
            return self.port_socket_sensor

    def getPortSockePublisher(self):
        with self.mutex_setting:
            return self.port_socket_joint_publisher

    def getPortSocketJoint(self):
        with self.mutex_setting:
            return self.port_socket_joint_states

    def changeSocketSettings(self):
        self.changeIpSocket()
        self.changePortSocketRobot()
        self.changePortSocketSensor()
        self.changePortSockePublisher()
        self.changePortSocketJoint()
        self.flag_change_settings = True

    def changeIpSocket(self):
        with self.mutex_setting:
            self.ip_socket = self.data_ip_input.text()

    def changePortSocketRobot(self):
        with self.mutex_setting:
            self.port_socket_robot = self.data_port_robot_input.text()

    def changePortSocketSensor(self):
        with self.mutex_setting:
            self.port_socket_sensor = self.data_port_sensor_input.text()

    def changePortSockePublisher(self):
        with self.mutex_setting:
            self.port_socket_joint_publisher = self.data_port_publish_input.text()

    def changePortSocketJoint(self):
        with self.mutex_setting:
            self.port_socket_joint_states = self.data_port_joint_input.text()

    ####################

    def setFlagPortCamera(self, bool_):
        with self.mutex_port_camera:
            self.flag_change_port_camera = bool_

    def setFlagDelayPaint(self, bool_):
        with self.mutex_delay_paint:
            self.flag_change_delay_paint = bool_

    def getFlagPortCamera(self):
        with self.mutex_port_camera:
            return self.flag_change_port_camera

    def getFlagDelayPaint(self):
        with self.mutex_delay_paint:
            return self.flag_change_delay_paint

    def getPortCamera(self):
        with self.mutex_port_camera:
            return self.port_camera

    def getDelayPaint(self):
        with self.mutex_delay_paint:
            return self.delay_paint

    def changePortCamera(self):
        with self.mutex_port_camera:
            self.port_camera = self.camera_port_input.text()
        self.setFlagPortCamera(True)

    def changeDelayPaint(self):
        with self.mutex_delay_paint:
            self.delay_paint = self.delay_paint_input.text()
        self.setFlagDelayPaint(True)

    #######################

    def setFlagChangeTimeout(self, bool_):
        with self.mutex_flag_change_timeout:
            self.flag_change_timeout = bool_

    def getFlagChangeTimeout(self):
        with self.mutex_flag_change_timeout:
            return self.flag_change_timeout

    def changeTimeoutSocketRobor(self):
        with self.mutex_change_timeout:
            self.socket_delay_robot = self.delay_socket_input.text()

    def changeTimeoutSocketSensor(self):
        with self.mutex_change_timeout:
            self.socket_delay_sensor = self.delay_sensor_input.text()

    def changeTimeoutSocketPubllisher(self):
        with self.mutex_change_timeout:
            self.socket_delay_publisher = self.delay_publisher_input.text()

    def changeTimeoutSocketJoints(self):
        with self.mutex_change_timeout:
            self.socket_delay_joint_states = self.delay_joint_input.text()

    def getTimeoutSocketRobor(self):
        with self.mutex_change_timeout:
            return self.socket_delay_robot

    def getTimeoutSocketSensor(self):
        with self.mutex_change_timeout:
            return self.socket_delay_sensor

    def getTimeoutSocketPubllisher(self):
        with self.mutex_change_timeout:
            return self.socket_delay_publisher

    def getTimeoutSocketJoints(self):
        with self.mutex_change_timeout:
            return self.socket_delay_joint_states

    def changeTimeoutSocketSettings(self):
        print(1)
        self.changeTimeoutSocketJoints()
        self.changeTimeoutSocketPubllisher()
        self.changeTimeoutSocketSensor()
        self.changeTimeoutSocketRobor()
        self.setFlagChangeTimeout(True)

    def incrementTable(self, key, number):
        for i, name in enumerate(self.table_names):
            if name == key:
                self.table_counter[i] = self.table_counter[i] + number
        self.fillTableStatistics()

    # *********************
    #    Парсер кнопочки  *
    # *********************

    def openFileDXF(self):
        options = QFileDialog.Options()
        file_path, _ = QFileDialog.getOpenFileName(
            self,
            "Открыть DXF файл",
            "",
            "DXF Files (*.dxf);;All Files (*)",
            options=options,
        )
        if file_path:
            print(
                f"Файл выбран: {file_path}"
            )  # Проверяем, что файл загружается корректно
            try:
                self.loadAndDisplayDXF(file_path)  # Метод обработки DXF
            except Exception as e:
                self.message_parser_dxf.setText(f"Ошибка при обработке файла: {e}")
                return
        else:
            self.message_parser_dxf.setText("Файл не выбран")
            return
        self.message_parser_dxf.setText("DXF парсер успешно открыт")

    def loadAndDisplayDXF(self, dxf_path):
        image_path = "temp.png"
        self.saveDXFToImage(dxf_path, image_path)

        img = mpimg.imread(image_path)
        self.ax.clear()
        self.ax.imshow(img.transpose((1, 0, 2)))
        self.ax.axis("off")
        self.clearLinesAndPoints()
        self.cid_click = self.dxf_canvas.mpl_connect(
            "button_press_event", self.on_click
        )
        self.cid_move = self.dxf_canvas.mpl_connect("motion_notify_event", self.on_move)
        self.dxf_canvas.draw()

    def endTrajectoryDXF(self):
        if len(self.points) > 1 and self.start_point is not None:
            # Замыкаем траекторию
            self.ax.plot(
                [self.last_point[0], self.start_point[0]],
                [self.last_point[1], self.start_point[1]],
                "r-",
                lw=2,
            )
            self.dxf_canvas.draw()

        # Сброс для новой траектории
        self.start_point = None
        self.last_point = None
        self.message_parser_dxf.setText("Траектория закончена")

    def saveTrajectoryDXF(self):
        """Сохраняет траекторию в CSV файл с координатами."""
        if len(self.points) > 0:
            z_value = 10  # self.z_input.text()  # Получаем значение Z из поля ввода
            try:
                z_value = float(z_value)  # Пробуем преобразовать Z в число
            except ValueError:
                self.message_parser_dxf.setText("Некорректное значение Z.")
                return
            file_name, _ = QFileDialog.getSaveFileName(
                self, "Сохранить траекторию", "", "CSV Files (*.csv)"
            )
            if file_name:
                with open(file_name, mode="w", newline="") as file:
                    writer = csv.writer(file)
                    writer.writerow(["X", "Y", "Z"])  # Заголовки
                    # Масштабируем координаты перед записью
                    scaled_points = [
                        (x * self.scale_factor_x, y * self.scale_factor_y, z_value)
                        for x, y in self.points
                    ]
                    writer.writerows(scaled_points)
        print("Сохранили")
        self.message_parser_dxf.setText("Траектория успешно сохранена")

    def clearTrajectoryDXF(self):
        self.message_parser_dxf.setText("Отчистки нет, загрузите dxf ещё раз")

    def sendTrajectoryDXF(self):
        print("Отправили")

    # ******************************
    #    Парсер рисуем траекторию  *
    # ******************************

    def on_click(self, event):
        if event.inaxes is None:
            return
        if event.button == 1:  # Левый клик
            point = (event.xdata, event.ydata)  # Координаты без масштабирования
            if self.start_point is None:
                # Начало новой линии
                self.start_point = point
                self.points.append(point)
            else:
                # Завершение линии
                line_plot = self.ax.plot(
                    [self.last_point[0], point[0]],
                    [self.last_point[1], point[1]],
                    "r-",
                    lw=2,
                )[0]
                self.line_plots.append(line_plot)
                self.points.append(point)
                self.dxf_canvas.draw()
            self.last_point = point

    def on_move(self, event):
        if event.inaxes is None:
            return

        # Отображаем координаты курсора с учётом масштаба
        scaled_x = (
            event.xdata * self.scale_factor_x if event.xdata is not None else None
        )
        scaled_y = (
            event.ydata * self.scale_factor_y if event.ydata is not None else None
        )
        self.coordinates_label.setText(
            f"Координаты курсора: X={scaled_x:.2f}, Y={scaled_y:.2f}"
            if scaled_x and scaled_y
            else "Координаты курсора: Вне области"
        )

        if self.start_point is None:
            return
        self.update_temp_line(event)

    def update_temp_line(self, event):
        if self.temp_line is not None:
            self.temp_line.remove()
        if self.start_point is not None:
            self.temp_line = self.ax.plot(
                [self.last_point[0], event.xdata],
                [self.last_point[1], event.ydata],
                "r--",
                lw=1,
            )[0]
            self.dxf_canvas.draw()

    def close_path(self):
        if len(self.points) > 1 and self.start_point is not None:
            # Замыкаем траекторию
            self.ax.plot(
                [self.last_point[0], self.start_point[0]],
                [self.last_point[1], self.start_point[1]],
                "r-",
                lw=2,
            )
            self.dxf_canvas.draw()

        # Сброс для новой траектории
        self.start_point = None
        self.last_point = None

    # *************************
    #    Доп функции парсера  *
    # *************************

    def saveDXFToImage(self, dxf_path, image_path):
        ...
        # # Сохраняет содержимое DXF файла в изображение.
        # doc, auditor = recover.readfile(dxf_path)
        # if not auditor.has_errors:
        #     matplotlib.qsave(doc.modelspace(), image_path)

    def clearLinesAndPoints(self):
        # Очищает линии и точки, но оставляет изображение.
        for line_plot in self.line_plots:
            line_plot.remove()
        self.line_plots.clear()
        self.points = []
        self.start_point = None
        self.last_point = None
        if self.temp_line is not None:
            self.temp_line.remove()
        self.temp_line = None

    # ********************
    #    Мигание сброса  *
    # ********************
    def blinkinButtonResetError(self):
        current_style = self.reset_error.styleSheet()

        if current_style == Path("gss/reset_error_error.qss").read_text():
            self.reset_error.setStyleSheet(Path("gss/reset_error.qss").read_text())
        else:
            self.reset_error.setStyleSheet(
                Path("gss/reset_error_error.qss").read_text()
            )

    # ****************
    #    Светофор    *
    # ****************

    def trafficLightSignal(self, color):
        if color == "red":
            self.signal_green.set_color(QColor(69, 69, 69))
            self.signal_red.set_color(QColor(242, 80, 110))
            self.signal_yellow.set_color(QColor(69, 69, 69))

        if color == "yellow":
            self.signal_green.set_color(QColor(69, 69, 69))
            self.signal_yellow.set_color(QColor(234, 242, 80))
            self.signal_red.set_color(QColor(69, 69, 69))
        if color == "green":
            self.signal_green.set_color(QColor(80, 242, 137))
            self.signal_yellow.set_color(QColor(69, 69, 69))
            self.signal_red.set_color(QColor(69, 69, 69))

    # ******************
    #    Сбросы        *
    # ******************

    def resetFlags(self):
        with self.mutex_reset:
            self.flag_start_programm = False
            self.flag_stop_programm = False
            self.flag_error = False

            self.mark_manual = ""

    def resetError(self):
        with self.mutex:
            self.flag_reset_error = True
        self.timer_reset_error.stop()
        self.reset_error.setStyleSheet(Path("gss/reset_error.qss").read_text())

    def addLogForOperator(self, message, level):
        with self.mutex_log:
            self.log_plain_text.add_log(level, message)


# if __name__ == "__main__":
#     app = QApplication(sys.argv)
#     main_window = App()

#     sys.exit(app.exec_())
