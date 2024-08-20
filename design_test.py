from PyQt5 import uic
from PyQt5.QtWidgets import QApplication, QGraphicsScene
from rov_control import rovAracUart, rovAracSwd
from PyQt5.QtCore import QTimer, QThread, Qt
from PyQt5.QtGui import QFont, QImage, QPixmap

Form, Window = uic.loadUiType("grov_interface_design.ui")

app = QApplication([])
window = Window()
form = Form()
form.setupUi(window)
window.show()

app.exec()
