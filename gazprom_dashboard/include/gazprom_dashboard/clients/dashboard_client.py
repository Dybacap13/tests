import tools.trigger_service as trigger_service
from rclpy.node import Node
from example_interfaces.srv import Trigger

from PyQt5.QtGui import QPixmap

import app.dashboard as dashboard


import device.camera as camera
import device.display as display
class DashboardClient:
  def __init__(self, port_camera):
     
      node = Node("dashboard_client")

      self.dashboard_ = dashboard.App() # передать все параметры
      self.camera_ = camera.ThreadOpenCV(port_camera)
      self.display_ = display.BackendThreadUpdateDisplay()



      self.dashboard_.connectDisplay(self.display_)



  # ******************
  #    С камерой     *
  # ******************

  def startCameraWork(self):
    self.camera_.changePixmap.connect(self.setImageCamera)
    self.camera_.start()
  
  def setImageCamera(self, image):
    self.dashboard_.label_video.setPixmap(QPixmap.fromImage(image))

  def stopCameraWork(self):
    self.camera_.setFlagStop(True)


