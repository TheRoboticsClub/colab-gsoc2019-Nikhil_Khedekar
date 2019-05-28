# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui_gui.ui'
#
# Created by: PyQt5 UI code generator 5.5.1
#
# WARNING! All changes made in this file will be lost!

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(660, 370)
        MainWindow.setMinimumSize(QtCore.QSize(660, 370))
        MainWindow.setMaximumSize(QtCore.QSize(660, 370))
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.takeoffButton = QtWidgets.QPushButton(self.centralwidget)
        self.takeoffButton.setGeometry(QtCore.QRect(470, 30, 161, 41))
        self.takeoffButton.setObjectName("takeoffButton")
        self.altdSlider = QtWidgets.QSlider(self.centralwidget)
        self.altdSlider.setGeometry(QtCore.QRect(400, 30, 19, 311))
        self.altdSlider.setMaximum(100)
        self.altdSlider.setProperty("value", 49)
        self.altdSlider.setOrientation(QtCore.Qt.Vertical)
        self.altdSlider.setObjectName("altdSlider")
        self.playstopButton = QtWidgets.QPushButton(self.centralwidget)
        self.playstopButton.setGeometry(QtCore.QRect(470, 80, 90, 51))
        self.icon = QtGui.QIcon()
        self.icon.addPixmap(QtGui.QPixmap(":/images/play.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.playstopButton.setIcon(self.icon)
        self.playstopButton.setObjectName("playstopButton")
        self.playstopButton.setCheckable(True)  
        self.playstopButton.setChecked(True)  
        self.playstopButton.toggle()
        #self.stopButton = QtWidgets.QPushButton(self.centralwidget)
        #self.stopButton.setGeometry(QtCore.QRect(560, 80, 71, 51))
        self.icon1 = QtGui.QIcon()
        self.icon1.addPixmap(QtGui.QPixmap(":/images/stop.png"), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        #self.stopButton.setIcon(icon1)
        #self.stopButton.setObjectName("stopButton")
        self.windowsLabel = QtWidgets.QLabel(self.centralwidget)
        self.windowsLabel.setGeometry(QtCore.QRect(540, 190, 71, 21))
        self.windowsLabel.setObjectName("windowsLabel")
        self.cameraCheck = QtWidgets.QCheckBox(self.centralwidget)
        self.cameraCheck.setGeometry(QtCore.QRect(540, 220, 94, 26))
        self.cameraCheck.setObjectName("cameraCheck")
        self.sensorsCheck = QtWidgets.QCheckBox(self.centralwidget)
        self.sensorsCheck.setGeometry(QtCore.QRect(540, 250, 94, 26))
        self.sensorsCheck.setObjectName("sensorsCheck")
        self.colorFilterCheck = QtWidgets.QCheckBox(self.centralwidget)
        self.colorFilterCheck.setGeometry(QtCore.QRect(540, 280, 94, 26))
        self.colorFilterCheck.setObjectName("colorFilterCheck")
        self.altdLabel = QtWidgets.QLabel(self.centralwidget)
        self.altdLabel.setGeometry(QtCore.QRect(390, 340, 51, 21))
        self.altdLabel.setObjectName("altdLabel")
        self.verticalLayoutWidget = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget.setGeometry(QtCore.QRect(20, 30, 361, 301))
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.tlLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget)
        self.tlLayout.setObjectName("tlLayout")
        self.rotationDial = QtWidgets.QDial(self.centralwidget)
        self.rotationDial.setGeometry(QtCore.QRect(440, 220, 50, 64))
        self.rotationDial.setMaximum(100)
        self.rotationDial.setProperty("value", 49)
        self.rotationDial.setObjectName("rotationDial")
        self.rotationLabel = QtWidgets.QLabel(self.centralwidget)
        self.rotationLabel.setGeometry(QtCore.QRect(440, 280, 65, 21))
        self.rotationLabel.setObjectName("rotationLabel")
        self.XLabel = QtWidgets.QLabel(self.centralwidget)
        self.XLabel.setGeometry(QtCore.QRect(20, 340, 21, 21))
        self.XLabel.setObjectName("XLabel")
        self.YLabel = QtWidgets.QLabel(self.centralwidget)
        self.YLabel.setGeometry(QtCore.QRect(130, 340, 21, 21))
        self.YLabel.setObjectName("YLabel")
        self.XValue = QtWidgets.QLabel(self.centralwidget)
        self.XValue.setGeometry(QtCore.QRect(40, 340, 41, 21))
        self.XValue.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.XValue.setObjectName("XValue")
        self.YValue = QtWidgets.QLabel(self.centralwidget)
        self.YValue.setGeometry(QtCore.QRect(150, 340, 41, 21))
        self.YValue.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.YValue.setObjectName("YValue")
        self.altdValue = QtWidgets.QLabel(self.centralwidget)
        self.altdValue.setGeometry(QtCore.QRect(390, 10, 41, 21))
        self.altdValue.setAlignment(QtCore.Qt.AlignCenter)
        self.altdValue.setObjectName("altdValue")
        self.rotValue = QtWidgets.QLabel(self.centralwidget)
        self.rotValue.setGeometry(QtCore.QRect(445, 200, 41, 21))
        self.rotValue.setAlignment(QtCore.Qt.AlignCenter)
        self.rotValue.setObjectName("rotValue")
        self.resetButton = QtWidgets.QPushButton(self.centralwidget)
        self.resetButton.setGeometry(QtCore.QRect(470, 140, 161, 41))
        self.resetButton.setObjectName("resetButton")
        self.verticalLayoutWidget_2 = QtWidgets.QWidget(self.centralwidget)
        self.verticalLayoutWidget_2.setGeometry(QtCore.QRect(590, 300, 61, 61))
        self.verticalLayoutWidget_2.setObjectName("verticalLayoutWidget_2")
        self.logoLayout = QtWidgets.QVBoxLayout(self.verticalLayoutWidget_2)
        self.logoLayout.setSpacing(0)
        self.logoLayout.setObjectName("logoLayout")
        MainWindow.setCentralWidget(self.centralwidget)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "Follow Turtlebot"))
        self.takeoffButton.setText(_translate("MainWindow", "Take off"))
        self.playstopButton.setText(_translate("MainWindow", "Play Code"))
        #self.stopButton.setText(_translate("MainWindow", "Stop Code"))
        self.windowsLabel.setText(_translate("MainWindow", "Windows:"))
        self.cameraCheck.setText(_translate("MainWindow", "Camera"))
        self.sensorsCheck.setText(_translate("MainWindow", "Sensors"))
        self.colorFilterCheck.setText(_translate("MainWindow", "Color filter"))
        self.altdLabel.setText(_translate("MainWindow", "Altitude"))
        self.rotationLabel.setText(_translate("MainWindow", "Rotation"))
        self.XLabel.setText(_translate("MainWindow", "X:"))
        self.YLabel.setText(_translate("MainWindow", "Y:"))
        self.XValue.setText(_translate("MainWindow", "0"))
        self.YValue.setText(_translate("MainWindow", "0"))
        self.altdValue.setText(_translate("MainWindow", "0"))
        self.rotValue.setText(_translate("MainWindow", "0"))
        self.resetButton.setText(_translate("MainWindow", "Reset"))

import resources_rc
