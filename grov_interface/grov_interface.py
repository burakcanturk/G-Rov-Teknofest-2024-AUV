from PyQt5 import uic
from PyQt5.QtWidgets import *
from rov_control import rovAracUart, rovAracSwd
from PyQt5.QtCore import QTimer, QThread, Qt
from PyQt5.QtGui import QFont, QImage, QPixmap
from time import sleep
import cv2
from threading import Thread
import os
from datetime import datetime
import numpy as np
import sys

def constrain(val, min_, max_):
    if val < min_:
        return min_
    elif val > max_:
        return max_
    else:
        return val

class MainWindow(QMainWindow):

    def __init__(self):

        super().__init__()
        uic.loadUi("grov_interface_design.ui", self)

        self.port = "/dev/ttyUSB0"
        self.baud = 115200

        self.grovUart = rovAracUart(port, baud)
        self.grovUart.adresAl(show = True)
        self.grovSwd = rovAracSwd()

        grov = {
            "UART": grovUart,
            "SWD": grovSwd
        }

        self.font = QFont('MS Shell Dlg 2', 14)

        self.DisFrontLabel.setAlignment(Qt.AlignCenter)
        self.DisBackLabel.setAlignment(Qt.AlignCenter)
        self.DisRightLabel.setAlignment(Qt.AlignCenter)
        self.DisLeftLabel.setAlignment(Qt.AlignCenter)
        self.DisLowerLabel.setAlignment(Qt.AlignCenter)

        self.DisFrontLabel.setFont(font)
        self.DisBackLabel.setFont(font)
        self.DisRightLabel.setFont(font)
        self.DisLeftLabel.setFont(font)
        self.DisLowerLabel.setFont(font)

        self.RollLabel.setFont(font)
        self.PitchLabel.setFont(font)
        self.YawLabel.setFont(font)

        self.VoltageLabel.setFont(font)
        self.CurrentLabel.setFont(font)
        self.PowerLabel.setFont(font)

        self.com_prtcl = "UART"

        self.motor_speed = 250
        self.rolicam_angle = 90
        self.rolicam_dim = 0

        self.frame = None

        self.camOn = True

        self.lanternOn = True

        self.keyPressed = False

        self.key_ = 0

        self.pressed = False
        self.released = False

        self.cam = cv2.VideoCapture(0)
        self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))

        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.thread = QThread()
        self.thread.run = self.getCamera
        self.thread.start()

        self.timerKeyboard = QTimer()
        self.timerKeyboard.timeout.connect(self.processKeyPress)
        self.timerKeyboard.start(100)

        self.timerGetData = QTimer()
        self.timerGetData.timeout.connect(self.getValues)
        self.timerGetData.start(100)

        self.CamOnCheckBox.stateChanged.connect(self.setCam)

        self.TakePhotoButton.clicked.connect(self.takePhoto)

        self.ComPrtclComboBox.currentIndexChanged.connect(self.setProtocol)

        self.GoForwardButton.pressed.connect(self.goForward)
        self.GoForwardButton.released.connect(self.stopLowerMotors)

        self.GoBackwardButton.pressed.connect(self.goBackward)
        self.GoBackwardButton.released.connect(self.stopLowerMotors)

        self.TurnRightButton.pressed.connect(self.turnRight)
        self.TurnRightButton.released.connect(self.stopLowerMotors)

        self.TurnLeftButton.pressed.connect(self.turnLeft)
        self.TurnLeftButton.released.connect(self.stopLowerMotors)

        self.GoRightButton.pressed.connect(self.goRight)
        self.GoRightButton.released.connect(self.stopLowerMotors)

        self.GoLeftButton.pressed.connect(self.goLeft)
        self.GoLeftButton.released.connect(self.stopLowerMotors)

        self.GoUpButton.pressed.connect(self.goUp)
        self.GoUpButton.released.connect(self.stopUpperMotors)

        self.GoDownButton.pressed.connect(self.goDown)
        self.GoDownButton.released.connect(self.stopUpperMotors)

        self.Tor0Button.pressed.connect(self.tor0On)
        self.Tor0Button.released.connect(self.tor0Off)

        self.Tor1Button.pressed.connect(self.tor1On)
        self.Tor1Button.released.connect(self.tor1Off)

        self.Tor2Button.pressed.connect(self.tor2On)
        self.Tor2Button.released.connect(self.tor2Off)

        self.Tor3Button.pressed.connect(self.tor3On)
        self.Tor3Button.released.connect(self.tor3Off)

        self.Tor4Button.pressed.connect(self.tor4On)
        self.Tor4Button.released.connect(self.tor4Off)

        self.MotorSpeedSlider.valueChanged.connect(self.setMotorSpeed)

        self.RolicamAngleSlider.valueChanged.connect(self.setRolicamAngle)

        self.LanternOnCheckBox.stateChanged.connect(self.setLantern)

        self.RolicamDimSlider.valueChanged.connect(self.setRolicamDim)

        self.RolicamSpeedSlider.valueChanged.connect(self.setRolicamSpeed)

        self.RolicamResetButton.pressed.connect(self.rolicamResetOn)
        self.RolicamResetButton.released.connect(self.rolicamResetOff)

        self.FilterResetButton.pressed.connect(self.filterResetOn)
        self.FilterResetButton.released.connect(self.filterResetOff)

    def takePhoto(self):
        if self.camOn:
            if self.frame is not None:
                if not os.path.exists("./photos"):
                    os.mkdir("photos")
                now = datetime.now()
                now_str = now.strftime("%d_%m_%Y %H_%M_%S")
                photo_name = f"{os.getcwd()}/photos/Foto {now_str}.jpg"
                cv2.imwrite(photo_name, self.frame)

    def setProtocol(self, index):
        self.self.com_prtcl = self.ComPrtclComboBox.itemText(index)
        if self.self.com_prtcl == "UART":
            self.grovUart.debugModeSet(0)
        elif self.self.com_prtcl == "SWD":
            self.grovUart.debugModeSet(1)

    def goForward(self):
        self.grov[self.com_prtcl].goForward(self.motor_speed, self.motor_speed)

    def goBackward(self):
        self.grov[self.com_prtcl].goBackward(self.motor_speed, self.motor_speed)

    def turnRight(self):
        self.grov[self.com_prtcl].turnRight(self.motor_speed)

    def turnLeft(self):
        self.grov[self.com_prtcl].turnLeft(self.motor_speed)

    def goRight(self):
        self.grov[self.com_prtcl].goRight(self.motor_speed, self.motor_speed)

    def goLeft(self):
        self.grov[self.com_prtcl].goLeft(self.motor_speed, self.motor_speed)

    def stopLowerMotors(self):
        self.grov[self.com_prtcl].stopLowerMotors()

    def goUp(self):
        self.grov[self.com_prtcl].goUp(self.motor_speed)

    def goDown(self):
       self.grov[self.com_prtcl].goDown(self.motor_speed)

    def stopUpperMotors(self):
        self.grov[self.com_prtcl].stopUpperMotors()

    def setMotorSpeed(self, val):
        self.MotorSpeedLabel.setText(str(val))
        self.motor_speed = val

    def tor0On():
        self.grov[self.com_prtcl].setRelay0(1)

    def tor0Off():
        self.grov[self.com_prtcl].setRelay0(0)

    def tor1On():
        self.grov[self.com_prtcl].setRelay1(1)

    def tor1Off():
        self.grov[self.com_prtcl].setRelay1(0)

    def tor2On():
        self.grov[self.com_prtcl].setRelay2(1)

    def tor2Off():
        self.grov[self.com_prtcl].setRelay2(0)

    def tor3On():
        self.grov[self.com_prtcl].setRelay3(1)

    def tor3Off():
        self.grov[self.com_prtcl].setRelay3(0)

    def tor4On():
        self.grov[self.com_prtcl].setRelay4(1)

    def tor4Off():
        self.grov[self.com_prtcl].setRelay4(0)

    def setRolicamAngle(self, val):
        self.RolicamAngleSliderTxt.setText(str(val))
        self.grov[self.com_prtcl].setRolicamAngle(val)
        self.rolicam_angle = val

    def setRolicamDim(self, val):
        if self.lanternOn:
            self.grov[self.com_prtcl].setRolicamDim(val)
        else:
            self.grov[self.com_prtcl].setRolicamDim(0)
        self.RolicamDimLabel.setText(str(val))
        self.rolicam_dim = val

    def setLantern(self, checked):
        self.lanternOn = bool(checked)
        if not self.lanternOn:
            self.grov[self.com_prtcl].setRolicamDim(0)
        else:
            self.grov[self.com_prtcl].setRolicamDim(self.rolicam_dim)
        self.RolicamDimSlider.setEnabled(self.lanternOn)

    def setRolicamSpeed(self, val):
        self.RolicamSpeedLabel.setText(str(val))
        self.grov[self.com_prtcl].rolicamSpeedSet(val)

    def rolicamResetOn(self):
        self.grov[self.com_prtcl].setRolicamReset(1)

    def rolicamResetOff(self):
        self.grov[self.com_prtcl].setRolicamReset(0)

    def filterResetOn(self):
        self.grov[self.com_prtcl].setFilterReset(1)

    def filterResetOff(self):
        self.grov[self.com_prtcl].setFilterReset(0)

    def getValues(self):

        self.DisFrontLabel.setText(str(self.grov[self.com_prtcl].getFrontDis()))
        self.DisBackLabel.setText(str(self.grov[self.com_prtcl].getBackDis()))
        self.DisRightLabel.setText(str(self.grov[self.com_prtcl].getRightDis()))
        self.DisLeftLabel.setText(str(self.grov[self.com_prtcl].getLeftDis()))
        self.DisLowerLabel.setText(str(self.grov[self.com_prtcl].getLowerDis()))

        self.RollLabel.setText(str(self.grov[self.com_prtcl].getRoll()))
        self.PitchLabel.setText(str(self.grov[self.com_prtcl].getPitch()))
        self.YawLabel.setText(str(self.grov[self.com_prtcl].getYaw()))

        self.VoltageLabel.setText(str(self.grov[self.com_prtcl].getVoltage()))
        self.CurrentLabel.setText(str(self.grov[self.com_prtcl].getCurrent()))
        self.PowerLabel.setText(str(self.grov[self.com_prtcl].getPower()))

    def processKeyPress(self):

        if self.keyPressed:
            
            if not self.pressed:

                if self.key_ < 0x110000:
                    self.key = chr(self.key_)

                else:
                    self.key = self.key_

                if self.key == "W":
                    self.grov[self.com_prtcl].goForward(self.motor_speed, self.motor_speed)

                elif self.key == "S":
                    self.grov[self.com_prtcl].goBackward(self.motor_speed, self.motor_speed)

                elif self.key == "D":
                    self.grov[self.com_prtcl].turnRight(self.motor_speed)

                elif self.key == "A":
                    self.grov[self.com_prtcl].turnLeft(self.motor_speed)

                elif self.key == "E":
                    self.grov[self.com_prtcl].goRight(self.motor_speed, self.motor_speed)

                elif self.key == "Q":
                    self.grov[self.com_prtcl].goLeft(self.motor_speed, self.motor_speed)

                if self.key == "O":
                    self.grov[self.com_prtcl].goUp(self.motor_speed)

                elif self.key == "L":
                    self.grov[self.com_prtcl].goDown(self.motor_speed)

                if self.key == "1":
                    self.grov[self.com_prtcl].setRelay0(1)

                if self.key == "2":
                    self.grov[self.com_prtcl].setRelay1(1)

                if self.key == "3":
                    self.grov[self.com_prtcl].setRelay2(1)

                if self.key == "4":
                    self.grov[self.com_prtcl].setRelay3(1)

                if self.key == "5":
                    self.grov[self.com_prtcl].setRelay4(1)

                if self.key == "+":
                    self.rolicam_angle += 10
                    self.rolicam_angle = constrain(self.rolicam_angle, 0, 180)
                    self.RolicamAngleSlider.setValue(self.rolicam_angle)

                if self.key == "-":
                    self.rolicam_angle -= 10
                    self.rolicam_angle = constrain(self.rolicam_angle, 0, 180)
                    self.RolicamAngleSlider.setValue(self.rolicam_angle)

                if self.key == Qt.Key_Enter - 1:
                    self.takePhoto()
                
                self.released = False
                self.pressed = True

        else:
            
            if not self.released:

                #print("Birakildi")

                if self.key_ < 0x110000:
                    self.key = chr(self.key_)

                else:
                    self.key = self.key_

                if self.key == "W" or self.key == "S" or self.key == "D" or self.key == "A" or self.key == "E" or self.key == "Q":
                    self.grov[self.com_prtcl].stopLowerMotors()

                if self.key == "O" or self.key == "L":
                    self.grov[self.com_prtcl].stopUpperMotors()

                if self.key == "1":
                    self.grov[self.com_prtcl].setRelay0(0)

                if self.key == "2":
                    self.grov[self.com_prtcl].setRelay1(0)

                if self.key == "3":
                    self.grov[self.com_prtcl].setRelay2(0)

                if self.key == "4":
                    self.grov[self.com_prtcl].setRelay3(0)

                if self.key == "5":
                    self.grov[self.com_prtcl].setRelay4(0)

                self.pressed = False
                self.released = True

        self.keyPressed = False

    def setCam(self, checked):
        self.camOn = bool(checked)
        self.TakePhotoButton.setEnabled(self.camOn)

    def getCamera():

        cam_opened = True
        cam_closed = False

        while True:

            if self.camOn:

                if not cam_opened:
                    self.cam = cv2.VideoCapture(0)
                    self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
                    self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    self.cam_closed = False
                    self.cam_opened = True

                ret, self.frame = cam.read()

                if ret:

                    self.frame = cv2.resize(self.frame, (640, 480))

                    frame_rgb = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)

                    h, w, ch = frame_rgb.shape
                    bytes_per_line = ch * w
                    convert_to_Qt_selfat = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.selfat_RGB888)
                    p = convert_to_Qt_selfat.scaled(640, 480, Qt.KeepAspectRatio)

                    self.CameraView.setPixmap(QPixmap.fromImage(p))

            else:

                if not cam_closed:
                    self.cam.release()
                    cam_opened = False
                    cam_closed = True

                frame_rgb = np.zeros((480, 640, 3), dtype = np.uint8)

                frame_rgb[:,:,:] = 255

                h, w, ch = frame_rgb.shape
                bytes_per_line = ch * w
                convert_to_Qt_selfat = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.selfat_RGB888)
                p = convert_to_Qt_selfat.scaled(640, 480, Qt.KeepAspectRatio)

                self.CameraView.setPixmap(QPixmap.fromImage(p))

    def keyPressEvent(self, event):
        self.keyPressed = True
        self.key_ = event.key()
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        self.keyPressed = False
        super().keyPressEvent(event)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
