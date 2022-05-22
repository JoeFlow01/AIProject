import os

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QUrl
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QMessageBox
from pyvis.network import Network
from PyQt5 import QtWebEngineWidgets

import Graph
from Graph import Graph

g = Graph()


class Ui_Form(object):
    def setupUi(self, Form):
        Form.setObjectName("Form")
        Form.resize(1080, 539)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        Form.setWindowIcon(QtGui.QIcon('icon.png'))
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Form.sizePolicy().hasHeightForWidth())
        Form.setSizePolicy(sizePolicy)
        Form.setMinimumSize(QtCore.QSize(1100, 600))
        Form.setMaximumSize(QtCore.QSize(1100, 600))
        Form.setStyleSheet(
            "QWidget#frmLogin,QWidget#frmPopup,QWidget#frmHostInfo,QWidget#frmLogout,QWidget#frmConfig,QWidget#frmData,QWidget#frmDefence,QWidget#frmHost,QWidget#frmMain,QWidget#frmPwd,QWidget#frmSelect,QWidget#frmMessageBox{\n"
            "    border:1px solid #0F7DBE;\n"
            "    border-radius:0px;    \n"
            "}\n"
            "\n"
            ".QFrame{\n"
            "    border:1px solid #50A3F0;\n"
            "    border-radius:5px;\n"
            "}\n"
            "\n"
            "QWidget#widget_title{\n"
            "    background: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 #0F7DBE, stop:1 #1582C3);\n"
            "}\n"
            "\n"
            "QLabel#lab_Ico,QLabel#lab_Title{\n"
            "    border-radius:0px;\n"
            "    color: #F0F0F0;\n"
            "    background-color:rgba(0,0,0,0);\n"
            "    border-style:none;\n"
            "}\n"
            "\n"
            "QLineEdit {\n"
            "    border: 1px solid #50A3F0;\n"
            "    border-radius: 5px;\n"
            "    padding: 2px;\n"
            "    background: none;\n"
            "    selection-background-color: #0F7DBE;\n"
            "}\n"
            "\n"
            "QLineEdit[echoMode=\"2\"] { \n"
            "    lineedit-password-character: 9679; \n"
            "}\n"
            "\n"
            ".QPushButton{\n"
            "    border-style: none;\n"
            "    border: 0px;\n"
            "    color: #F0F0F0;\n"
            "    padding: 5px;    \n"
            "    min-height: 20px;\n"
            "    border-radius:5px;\n"
            "    background: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 #0F7DBE, stop:1 #1582C3); \n"
            "}\n"
            "\n"
            ".QPushButton[focusPolicy=\"0\"] {\n"
            "    border-style: none;\n"
            "    border: 0px;\n"
            "    color: #F0F0F0;\n"
            "    padding: 0px;    \n"
            "    min-height: 10px;\n"
            "    border-radius:3px;\n"
            "    background: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 #0F7DBE, stop:1 #1582C3); \n"
            "}\n"
            "\n"
            ".QPushButton:hover{ \n"
            "    background: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 #50A3F0, stop:1 #489CEA);\n"
            "}\n"
            "\n"
            ".QPushButton:pressed{ \n"
            "    background: qlineargradient(spread:pad, x1:0, y1:0, x2:0, y2:1, stop:0 #0F7DBE, stop:1 #1582C3);\n"
            "}\n"
            "\n"
            "QPushButton#btnMenu,QPushButton#btnMenu_Min,QPushButton#btnMenu_Max,QPushButton#btnMenu_Close{\n"
            "    border-radius:0px;\n"
            "    color: #F0F0F0;\n"
            "    background-color:rgba(0,0,0,0);\n"
            "    border-style:none;\n"
            "}\n"
            "\n"
            "QPushButton#btnMenu:hover,QPushButton#btnMenu_Min:hover,QPushButton#btnMenu_Max:hover{\n"
            "    background-color: qlineargradient(spread:pad, x1:0, y1:1, x2:0, y2:0, stop:0 rgba(25, 134, 199, 0), stop:1 #50A3F0);\n"
            "}\n"
            "\n"
            "QPushButton#btnMenu_Close:hover{\n"
            "    background-color: qlineargradient(spread:pad, x1:0, y1:1, x2:0, y2:0, stop:0 rgba(238, 0, 0, 128), stop:1 rgba(238, 44, 44, 255));\n"
            "}\n"
            "\n")
        self.comboBox = QtWidgets.QComboBox(Form)
        self.comboBox.setGeometry(QtCore.QRect(930, 30, 150, 28))
        self.comboBox.setObjectName("comboBox")
        self.comboBox.addItem("")
        self.comboBox.addItem("")
        self.gridLayout_2 = QtWidgets.QGridLayout(Form)
        self.gridLayout_2.setObjectName("gridLayout_2")
        self.gridLayout = QtWidgets.QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.widget = QtWebEngineWidgets.QWebEngineView(Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.widget.sizePolicy().hasHeightForWidth())
        self.widget.setSizePolicy(sizePolicy)
        self.gridLayout.setAlignment(Qt.AlignCenter)
        self.widget.setMinimumSize(QtCore.QSize(525, 520))
        self.widget.setMaximumSize(QtCore.QSize(525, 520))
        self.widget.setObjectName("widget")
        self.gridLayout.addWidget(self.widget, 0, 0, 1, 1)
        self.tabWidget = QtWidgets.QTabWidget(Form)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.tabWidget.sizePolicy().hasHeightForWidth())
        self.tabWidget.setSizePolicy(QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed))
        self.tabWidget.setMinimumSize(QtCore.QSize(459, 520))
        self.tabWidget.setMaximumSize(QtCore.QSize(459, 520))
        self.gridLayout.setHorizontalSpacing(10)
        self.tabWidget.setObjectName("tabWidget")
        self.tab_5 = QtWidgets.QWidget()
        self.tab_5.setObjectName("tab_5")
        self.layoutWidget = QtWidgets.QWidget(self.tab_5)
        self.layoutWidget.setGeometry(QtCore.QRect(0, 0, 451, 491))
        self.layoutWidget.setObjectName("layoutWidget")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.layoutWidget)
        self.verticalLayout_3.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.lineEdit_9 = QtWidgets.QLineEdit(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit_9.sizePolicy().hasHeightForWidth())
        self.lineEdit_9.setSizePolicy(sizePolicy)
        self.lineEdit_9.setObjectName("lineEdit_9")
        self.verticalLayout_3.addWidget(self.lineEdit_9)
        self.pushButton_11 = QtWidgets.QPushButton(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_11.sizePolicy().hasHeightForWidth())
        self.pushButton_11.setSizePolicy(sizePolicy)
        self.pushButton_11.setObjectName("pushButton_11")
        self.verticalLayout_3.addWidget(self.pushButton_11)
        self.pushButton_12 = QtWidgets.QPushButton(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_12.sizePolicy().hasHeightForWidth())
        self.pushButton_12.setSizePolicy(sizePolicy)
        self.pushButton_12.setObjectName("pushButton_12")
        self.verticalLayout_3.addWidget(self.pushButton_12)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.lineEdit_10 = QtWidgets.QLineEdit(self.layoutWidget)
        self.lineEdit_37 = QtWidgets.QLineEdit(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit_10.sizePolicy().hasHeightForWidth())
        self.lineEdit_10.setSizePolicy(sizePolicy)
        self.lineEdit_10.setMinimumSize(QtCore.QSize(80, 0))
        self.lineEdit_10.setMaximumSize(QtCore.QSize(16666658, 50))
        self.lineEdit_10.setObjectName("lineEdit_10")
        self.horizontalLayout_3.addWidget(self.lineEdit_10)

        sizePolicy.setHeightForWidth(self.lineEdit_37.sizePolicy().hasHeightForWidth())
        self.lineEdit_37.setSizePolicy(sizePolicy)
        self.lineEdit_37.setMinimumSize(QtCore.QSize(80, 0))
        self.lineEdit_37.setMaximumSize(QtCore.QSize(16666658, 50))
        self.lineEdit_37.setObjectName("lineEdit_37")
        self.horizontalLayout_3.addWidget(self.lineEdit_37)


        self.lineEdit_11 = QtWidgets.QLineEdit(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit_11.sizePolicy().hasHeightForWidth())
        self.lineEdit_11.setSizePolicy(sizePolicy)
        self.lineEdit_11.setMinimumSize(QtCore.QSize(80, 0))
        self.lineEdit_11.setMaximumSize(QtCore.QSize(16666658, 50))
        self.lineEdit_11.setObjectName("lineEdit_11")
        self.horizontalLayout_3.addWidget(self.lineEdit_11)
        self.lineEdit_12 = QtWidgets.QLineEdit(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit_12.sizePolicy().hasHeightForWidth())
        self.lineEdit_12.setSizePolicy(sizePolicy)
        self.lineEdit_12.setMinimumSize(QtCore.QSize(80, 0))
        self.lineEdit_12.setMaximumSize(QtCore.QSize(16666658, 50))
        self.lineEdit_12.setObjectName("lineEdit_12")
        self.horizontalLayout_3.addWidget(self.lineEdit_12)
        self.verticalLayout_3.addLayout(self.horizontalLayout_3)
        self.pushButton_13 = QtWidgets.QPushButton(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_13.sizePolicy().hasHeightForWidth())
        self.pushButton_13.setSizePolicy(sizePolicy)
        self.pushButton_13.setObjectName("pushButton_13")
        self.verticalLayout_3.addWidget(self.pushButton_13)
        self.pushButton_14 = QtWidgets.QPushButton(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_14.sizePolicy().hasHeightForWidth())
        self.pushButton_14.setSizePolicy(sizePolicy)
        self.pushButton_14.setObjectName("pushButton_14")
        self.verticalLayout_3.addWidget(self.pushButton_14)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.lineEdit_13 = QtWidgets.QLineEdit(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit_13.sizePolicy().hasHeightForWidth())
        self.lineEdit_13.setSizePolicy(sizePolicy)
        self.lineEdit_13.setMinimumSize(QtCore.QSize(80, 0))
        self.lineEdit_13.setMaximumSize(QtCore.QSize(16666658, 50))
        self.lineEdit_13.setObjectName("lineEdit_13")
        self.horizontalLayout_4.addWidget(self.lineEdit_13)
        self.lineEdit_14 = QtWidgets.QLineEdit(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit_14.sizePolicy().hasHeightForWidth())
        self.lineEdit_14.setSizePolicy(sizePolicy)
        self.lineEdit_14.setMinimumSize(QtCore.QSize(80, 0))
        self.lineEdit_14.setMaximumSize(QtCore.QSize(16666658, 50))
        self.lineEdit_14.setText("")
        self.lineEdit_14.setObjectName("lineEdit_14")
        self.horizontalLayout_4.addWidget(self.lineEdit_14)
        self.verticalLayout_3.addLayout(self.horizontalLayout_4)
        self.pushButton_15 = QtWidgets.QPushButton(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_15.sizePolicy().hasHeightForWidth())
        self.pushButton_15.setSizePolicy(sizePolicy)
        self.pushButton_15.setObjectName("pushButton_15")
        self.verticalLayout_3.addWidget(self.pushButton_15)
        self.pushButton_16 = QtWidgets.QPushButton(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_16.sizePolicy().hasHeightForWidth())
        self.pushButton_16.setSizePolicy(sizePolicy)
        self.pushButton_16.setObjectName("pushButton_16")
        self.verticalLayout_3.addWidget(self.pushButton_16)
        self.pushButton_17 = QtWidgets.QPushButton(self.layoutWidget)
        self.pushButton_27 = QtWidgets.QPushButton(self.layoutWidget)
        self.pushButton_37 = QtWidgets.QPushButton(self.layoutWidget)
        self.lineEdit_37 = QtWidgets.QLineEdit(self.layoutWidget)
        self.showpath = QtWidgets.QPushButton(self.layoutWidget)
        self.Path = QtWidgets.QLineEdit(self.layoutWidget)
        self.Cost = QtWidgets.QLineEdit(self.layoutWidget)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_17.sizePolicy().hasHeightForWidth())
        sizePolicy.setHeightForWidth(self.pushButton_27.sizePolicy().hasHeightForWidth())
        sizePolicy.setHeightForWidth(self.pushButton_37.sizePolicy().hasHeightForWidth())
        sizePolicy.setHeightForWidth(self.lineEdit_37.sizePolicy().hasHeightForWidth())
        sizePolicy.setHeightForWidth(self.showpath.sizePolicy().hasHeightForWidth())
        sizePolicy.setHeightForWidth(self.Path.sizePolicy().hasHeightForWidth())
        sizePolicy.setHeightForWidth(self.Cost.sizePolicy().hasHeightForWidth())
        self.pushButton_17.setSizePolicy(sizePolicy)
        self.pushButton_27.setSizePolicy(sizePolicy)
        self.pushButton_37.setSizePolicy(sizePolicy)
        self.pushButton_37.setObjectName("pushButton_37")

        # self.lineEdit_37.sizePolicy(sizePolicy)
        self.showpath.setSizePolicy(sizePolicy)
        self.Path.setSizePolicy(sizePolicy)
        self.Cost.setSizePolicy(sizePolicy)
        self.pushButton_17.setObjectName("pushButton_17")
        self.showpath.setObjectName("showpath")
        self.Path.setObjectName("Path")
        self.Cost.setObjectName("Path")
        self.verticalLayout_3.addWidget(self.pushButton_17)
        self.verticalLayout_3.addWidget(self.pushButton_27)
        self.verticalLayout_3.addWidget(self.pushButton_37)
        self.verticalLayout_3.addWidget(self.lineEdit_37)
        self.verticalLayout_3.addWidget(self.showpath)
        self.verticalLayout_3.addWidget(self.Path)
        self.verticalLayout_3.addWidget(self.Cost)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.tab_6 = QtWidgets.QWidget()
        self.tab_6.setObjectName("tab_6")
        self.layoutWidget_2 = QtWidgets.QWidget(self.tab_6)
        self.layoutWidget_2.setGeometry(QtCore.QRect(0, 0, 451, 491))
        self.layoutWidget_2.setObjectName("layoutWidget_2")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.layoutWidget_2)
        self.verticalLayout_4.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.lineEdit_15 = QtWidgets.QLineEdit(self.layoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit_15.sizePolicy().hasHeightForWidth())
        self.lineEdit_15.setSizePolicy(sizePolicy)
        self.lineEdit_15.setObjectName("lineEdit_15")
        self.verticalLayout_4.addWidget(self.lineEdit_15)
        self.lineEdit_21 = QtWidgets.QLineEdit(self.layoutWidget_2)
        self.lineEdit_21.setSizePolicy(sizePolicy)
        self.lineEdit_21.setMinimumSize(QtCore.QSize(80, 0))
        self.lineEdit_21.setMaximumSize(QtCore.QSize(16666658, 50))
        self.lineEdit_21.setText("")
        self.lineEdit_21.setObjectName("lineEdit_21")
        self.verticalLayout_4.addWidget(self.lineEdit_21)
        self.pushButton_18 = QtWidgets.QPushButton(self.layoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_18.sizePolicy().hasHeightForWidth())
        self.pushButton_18.setSizePolicy(sizePolicy)
        self.pushButton_18.setObjectName("pushButton_18")
        self.verticalLayout_4.addWidget(self.pushButton_18)
        self.pushButton_19 = QtWidgets.QPushButton(self.layoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_19.sizePolicy().hasHeightForWidth())
        self.pushButton_19.setSizePolicy(sizePolicy)
        self.pushButton_19.setObjectName("pushButton_19")
        self.verticalLayout_4.addWidget(self.pushButton_19)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.lineEdit_16 = QtWidgets.QLineEdit(self.layoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit_16.sizePolicy().hasHeightForWidth())
        self.lineEdit_16.setSizePolicy(sizePolicy)
        self.lineEdit_16.setMinimumSize(QtCore.QSize(80, 0))
        self.lineEdit_16.setMaximumSize(QtCore.QSize(16666658, 50))
        self.lineEdit_16.setObjectName("lineEdit_16")
        self.horizontalLayout_5.addWidget(self.lineEdit_16)
        self.lineEdit_17 = QtWidgets.QLineEdit(self.layoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit_17.sizePolicy().hasHeightForWidth())
        self.lineEdit_17.setSizePolicy(sizePolicy)
        self.lineEdit_17.setMinimumSize(QtCore.QSize(80, 0))
        self.lineEdit_17.setMaximumSize(QtCore.QSize(16666658, 50))
        self.lineEdit_17.setObjectName("lineEdit_17")
        self.horizontalLayout_5.addWidget(self.lineEdit_17)
        self.lineEdit_18 = QtWidgets.QLineEdit(self.layoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit_18.sizePolicy().hasHeightForWidth())
        self.lineEdit_18.setSizePolicy(sizePolicy)
        self.lineEdit_18.setMinimumSize(QtCore.QSize(80, 0))
        self.lineEdit_18.setMaximumSize(QtCore.QSize(16666658, 50))
        self.lineEdit_18.setObjectName("lineEdit_18")
        self.horizontalLayout_5.addWidget(self.lineEdit_18)
        self.verticalLayout_4.addLayout(self.horizontalLayout_5)
        self.pushButton_20 = QtWidgets.QPushButton(self.layoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_20.sizePolicy().hasHeightForWidth())
        self.pushButton_20.setSizePolicy(sizePolicy)
        self.pushButton_20.setObjectName("pushButton_20")
        self.verticalLayout_4.addWidget(self.pushButton_20)
        self.pushButton_21 = QtWidgets.QPushButton(self.layoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_21.sizePolicy().hasHeightForWidth())
        self.pushButton_21.setSizePolicy(sizePolicy)
        self.pushButton_21.setObjectName("pushButton_21")
        self.verticalLayout_4.addWidget(self.pushButton_21)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit_21.sizePolicy().hasHeightForWidth())

        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setSizeConstraint(QtWidgets.QLayout.SetDefaultConstraint)
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.lineEdit_19 = QtWidgets.QLineEdit(self.layoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit_19.sizePolicy().hasHeightForWidth())
        self.lineEdit_19.setSizePolicy(sizePolicy)
        self.lineEdit_19.setMinimumSize(QtCore.QSize(80, 0))
        self.lineEdit_19.setMaximumSize(QtCore.QSize(16666658, 50))
        self.lineEdit_19.setObjectName("lineEdit_19")
        self.horizontalLayout_6.addWidget(self.lineEdit_19)
        self.lineEdit_20 = QtWidgets.QLineEdit(self.layoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.lineEdit_20.sizePolicy().hasHeightForWidth())
        self.lineEdit_20.setSizePolicy(sizePolicy)
        self.lineEdit_20.setMinimumSize(QtCore.QSize(80, 0))
        self.lineEdit_20.setMaximumSize(QtCore.QSize(16666658, 50))
        self.lineEdit_20.setText("")
        self.lineEdit_20.setObjectName("lineEdit_20")
        self.horizontalLayout_6.addWidget(self.lineEdit_20)
        self.verticalLayout_4.addLayout(self.horizontalLayout_6)
        self.pushButton_22 = QtWidgets.QPushButton(self.layoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_22.sizePolicy().hasHeightForWidth())
        self.pushButton_22.setSizePolicy(sizePolicy)
        self.pushButton_22.setObjectName("pushButton_22")
        self.verticalLayout_4.addWidget(self.pushButton_22)
        self.pushButton_23 = QtWidgets.QPushButton(self.layoutWidget_2)
        self.showpathinf = QtWidgets.QPushButton(self.layoutWidget_2)
        self.PathInf = QtWidgets.QLineEdit(self.layoutWidget_2)
        self.CostInf = QtWidgets.QLineEdit(self.layoutWidget_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pushButton_23.sizePolicy().hasHeightForWidth())
        sizePolicy.setHeightForWidth(self.showpathinf.sizePolicy().hasHeightForWidth())
        sizePolicy.setHeightForWidth(self.PathInf.sizePolicy().hasHeightForWidth())
        sizePolicy.setHeightForWidth(self.CostInf.sizePolicy().hasHeightForWidth())
        self.pushButton_23.setSizePolicy(sizePolicy)
        self.showpathinf.setSizePolicy(sizePolicy)
        self.PathInf.setSizePolicy(sizePolicy)
        self.CostInf.setSizePolicy(sizePolicy)
        self.pushButton_23.setObjectName("pushButton_23")
        self.showpathinf.setObjectName("showpathinf")
        self.PathInf.setObjectName("PathInf")
        self.CostInf.setObjectName("CostInf")
        self.verticalLayout_4.addWidget(self.pushButton_23)
        self.verticalLayout_4.addWidget(self.showpathinf)
        self.verticalLayout_4.addWidget(self.PathInf)
        self.verticalLayout_4.addWidget(self.CostInf)
        self.label_2 = QtWidgets.QLabel(self.layoutWidget_2)
        font = QtGui.QFont()
        font.setPointSize(10)
        self.label_2.setFont(font)
        self.label_2.setObjectName("label_2")
        self.verticalLayout_4.addWidget(self.label_2)
        self.tabWidget.addTab(self.tab_6, "")
        self.tabWidget.addTab(self.tab_5, "")
        self.gridLayout.addWidget(self.tabWidget, 0, 1, 1, 1)
        self.gridLayout_2.addLayout(self.gridLayout, 0, 0, 1, 1)
        self.Path.setStyleSheet("QPushButton { text-align: left;  }")
        self.Cost.setStyleSheet("QPushButton { text-align: left; }")
        self.CostInf.setStyleSheet("QPushButton { text-align: left; }")
        self.PathInf.setStyleSheet("QPushButton { text-align: left; }")
        self.PathInf.setEnabled(False)
        self.CostInf.setEnabled(False)
        self.Path.setEnabled(False)
        self.Cost.setEnabled(False)
        self.retranslateUi(Form)
        self.tabWidget.setCurrentIndex(0)
        self.pushButton_11.clicked.connect(lambda: self.insert_clicked(self.getText()))
        self.pushButton_18.clicked.connect(lambda: self.insert_Inf(self.getText15(), self.getText21()))
        self.pushButton_12.clicked.connect(lambda: self.delete(self.getText()))
        self.pushButton_19.clicked.connect(lambda: self.delete(self.getText15()))
        self.pushButton_13.clicked.connect(
            lambda: self.edge_clicked(self.getText10(), self.getText11(), self.getText12()))
        self.pushButton_20.clicked.connect(
            lambda: self.edge_clicked(self.getText16(), self.getText17(), self.getText18()))
        self.pushButton_16.clicked.connect(lambda: self.dfs(self.getText13(), self.getText14()))
        self.pushButton_37.clicked.connect(lambda: self.Depth_Limited(self.getText13(), self.getText14(),self.getText37()))
        self.pushButton_15.clicked.connect(lambda: self.bfs(self.getText13(), self.getText14()))
        self.pushButton_17.clicked.connect(lambda: self.ucs(self.getText13(), self.getText14()))
        self.pushButton_27.clicked.connect(lambda: self.Iterative_Deepening(self.getText13(), self.getText14()))
        self.pushButton_14.clicked.connect(lambda: self.delete_edge_directed(self.getText10(), self.getText11()))
        self.pushButton_21.clicked.connect(lambda: self.delete_edge_directed(self.getText16(), self.getText17()))
        self.pushButton_22.clicked.connect(lambda: self.greedy(self.getText19(), self.getText20()))
        self.pushButton_23.clicked.connect(lambda: self.aStar(self.getText19(), self.getText20()))
        self.showpath.clicked.connect(lambda: self.showpathf())
        self.showpathinf.clicked.connect(lambda: self.showpathinff())

        QtCore.QMetaObject.connectSlotsByName(Form)

    def retranslateUi(self, Form):
        _translate = QtCore.QCoreApplication.translate
        Form.setWindowTitle(_translate("Dialog", "Dialog"))
        self.comboBox.setItemText(0, _translate("Dialog", "Directed Graph"))
        self.comboBox.setItemText(1, _translate("Dialog", "Undirected Graph"))
        Form.setWindowTitle(_translate("Form", "CSE-472 Project"))
        self.lineEdit_9.setPlaceholderText(_translate("Form", "Vertex name to Insert or Delete"))
        self.pushButton_11.setText(_translate("Form", "Add Vertex"))
        self.pushButton_12.setText(_translate("Form", "Delete Vertex"))
        self.lineEdit_10.setPlaceholderText(_translate("Form", "Start"))
        self.lineEdit_37.setPlaceholderText(_translate("Form", "Enter level"))
        self.lineEdit_11.setPlaceholderText(_translate("Form", "End"))
        self.lineEdit_12.setPlaceholderText(_translate("Form", "Weight"))
        self.pushButton_13.setText(_translate("Form", "Add edge"))
        self.pushButton_14.setText(_translate("Form", "Remove Edge"))
        self.lineEdit_13.setPlaceholderText(_translate("Form", "Start"))
        self.lineEdit_14.setPlaceholderText(_translate("Form", "Goal(s)    a,b,c,..."))
        self.pushButton_15.setText(_translate("Form", "BFS"))
        self.pushButton_16.setText(_translate("Form", "DFS"))
        self.pushButton_17.setText(_translate("Form", "UCS"))
        self.pushButton_27.setText(_translate("Form", "Iterative Deepening"))
        self.pushButton_37.setText(_translate("Form", "Depth Limited"))
        self.showpath.setText(_translate("Form", "Show Path"))
        self.Path.setText(_translate("Form", "Path: "))
        self.Cost.setText(_translate("Form", "Cost: "))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_5), _translate("Form", "Uninformed Search"))
        self.lineEdit_15.setPlaceholderText(_translate("Form", "Vertex name to Insert or Delete"))
        self.lineEdit_21.setPlaceholderText(_translate("Form", "Heuristic Value"))
        self.pushButton_18.setText(_translate("Form", "Add Vertex"))
        self.pushButton_19.setText(_translate("Form", "Delete Vertex"))
        self.lineEdit_16.setPlaceholderText(_translate("Form", "Start"))
        self.lineEdit_17.setPlaceholderText(_translate("Form", "End"))
        self.lineEdit_18.setPlaceholderText(_translate("Form", "Weight"))
        self.pushButton_20.setText(_translate("Form", "Add edge"))
        self.pushButton_21.setText(_translate("Form", "Remove Edge"))
        self.lineEdit_19.setPlaceholderText(_translate("Form", "Start"))
        self.lineEdit_20.setPlaceholderText(_translate("Form", "Goal(s)    a,b,c,..."))
        self.pushButton_22.setText(_translate("Form", "Greedy"))
        self.pushButton_23.setText(_translate("Form", "A*"))
        self.showpathinf.setText(_translate("Form", "Show Path"))
        self.PathInf.setText(_translate("Form", "Path: "))
        self.CostInf.setText(_translate("Form", "Cost:"))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_6), _translate("Form", "Informed Search"))

    l = []
    cost = 0

    def greedy(self, txt, tx2):
        g.remove_pref()
        try:
            g.explored.clear()
            w = tx2.split(",")
            self.l, self.cost = g.greedy(txt, w)
            self.visualize([], g.explored)
            self.changePathinf(self.l, self.cost)
        except:
            self.PathInf.setText("NO PATH AVAILABLE")

    def aStar(self, txt, tx2):
        g.remove_pref()
        try:
            g.explored.clear()
            w = tx2.split(",")
            self.l, self.cost = g.aStar(txt, w)
            self.visualize([], g.explored)
            self.changePathinf(self.l, self.cost)
        except:
            self.Path.setText("NO PATH AVAILABLE")

    def insert_Inf(self, txt, tx2):
        if txt == '':
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Vertex Name!")
            msg.setInformativeText("Vertex Name can't be null")
            msg.setWindowTitle("Error")
            msg.exec_()
            return
        elif tx2 == '':
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Heuristic Value!")
            msg.setInformativeText("You MUST add Heuristic Value")
            msg.setWindowTitle("Error")
            msg.exec_()
            return
        elif tx2.isdigit() is False:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Heuristic Value!")
            msg.setInformativeText("Heuristic Value MUST be Numeric")
            msg.setWindowTitle("Error")
            msg.exec_()
            return
        t = g.insertVertex(txt, int(tx2))
        if t == -1:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Vertex Already Exists!")
            msg.setInformativeText("No Duplicates allowed")
            msg.setWindowTitle("Error")
            msg.exec_()
            return
        self.drawGraph()
        self.lineEdit_21.clear()
        self.lineEdit_15.clear()

    def getText19(self):
        return self.lineEdit_19.text()

    def getText20(self):
        return self.lineEdit_20.text()

    def getText(self):
        return self.lineEdit_9.text()

    def getText15(self):
        return self.lineEdit_15.text()

    def getText10(self):
        return self.lineEdit_10.text()

    def getText11(self):
        return self.lineEdit_11.text()

    def getText12(self):
        return self.lineEdit_12.text()

    def getText21(self):
        return self.lineEdit_21.text()

    def getText13(self):
        return self.lineEdit_13.text()

    def getText14(self):
        return self.lineEdit_14.text()

    def getText16(self):
        return self.lineEdit_16.text()

    def getText17(self):
        return self.lineEdit_17.text()
    def getText37(self):
        return self.lineEdit_37.text()

    def getText18(self):
        return self.lineEdit_18.text()

    def delete_edge_directed(self, txt, tx2):

        if txt == '' or tx2 == '':
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Edge Error!")
            msg.setInformativeText("Start or End can't be null")
            msg.setWindowTitle("Error")
            msg.exec_()
            return
        t = g.deleteEdge_undirected(txt, tx2)
        if t == "edge doesn't exist":
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Edge Error!")
            msg.setInformativeText("Edge doesn't exist")
            msg.setWindowTitle("Error")
            msg.exec_()
            return
        elif t == "v doesn't exist":
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Edge Error!")
            msg.setInformativeText("Vertex Doesn't Exist")
            msg.setWindowTitle("Error")
            msg.exec_()
            return
        self.drawGraph()
        self.lineEdit_10.clear()
        self.lineEdit_11.clear()
        self.lineEdit_12.clear()
        self.lineEdit_16.clear()
        self.lineEdit_17.clear()
        self.lineEdit_18.clear()


    #function to delete undirected edge
    def delete_edge_undirected(self, txt, tx2):
        self.delete_edge_directed(txt, tx2)
        self.delete_edge_directed(tx2, txt)


    def dfs(self, txt, tx2):
        g.remove_pref()
        try:
            g.explored.clear()
            w = tx2.split(",")
            self.l, self.cost = g.DFS(txt, w)
            self.visualize([], g.explored)
            self.changePath(self.l, self.cost)
        except:
            self.Path.setText("NO PATH AVAILABLE")

    def Depth_Limited(self, txt, tx2,limit):

        #int(limit)
        # print("@GUI")
        g.remove_pref()
        try:
            g.explored.clear()
            w = tx2.split(",")
            # print(limit)
            self.l, self.cost = g.DFS_Limited(txt, w,limit)

            self.visualize([], g.explored)
            self.changePath(self.l, self.cost)
        except:
            self.Path.setText("NO PATH AVAILABLE")
    def Iterative_Deepening(self, txt, tx2):
        g.remove_pref()
        try:
            g.explored.clear()
            w = tx2.split(",")
            #print("hi")
            self.l, self.cost = g.BFS(txt, w)
            self.visualize([], g.explored)
            self.changePath(self.l, self.cost)
        except:
            self.Path.setText("NO PATH AVAILABLE")

    def ucs(self, txt, tx2):
        g.remove_pref()
        try:
            g.explored.clear()
            w = tx2.split(",")
            self.l, self.cost = g.UCS(txt, w)
            self.visualize([], g.explored)
            self.changePath(self.l, self.cost)
        except:
            self.Path.setText("NO PATH AVAILABLE")

    def changePathinf(self, lis, c):
        st = ""
        for i in lis:
            st += i
            st += ", "
        st1 = "Path: ["
        st1 += st
        st1 += "]"
        st2 = "Cost: "
        st2 += str(c)
        self.PathInf.setText(st1)
        self.CostInf.setText(st2)

    def bfs(self, txt, tx2):
        g.remove_pref()
        try:
            g.explored.clear()
            w = tx2.split(",")
            self.l, self.cost = g.BFS(txt, w)
            self.visualize([], g.explored)
            self.changePath(self.l, self.cost)
        except:
            self.Path.setText("NO PATH AVAILABLE")

    def showpathf(self):
        self.visualize(self.l, g.explored)

    def showpathinff(self):
        self.visualize(self.l, g.explored)

    def changePath(self, lis, c):
        st = ""
        for i in lis:
            st += i
            st += ", "
        st1 = "Path is: ["
        st1 += st
        st1 += "]"
        st2 = "with Cost:: "
        st2 += str(c)
        self.Path.setText(st1)
        self.Cost.setText(st2)

    def visualize(self, l, v):
        graphType = str(self.comboBox.currentText())
        bool=True
        if(graphType=="Undirected Graph" ):
            bool=False
        N = Network(height='100%', width='100%', directed=bool)
        for i in g.ListofVertices:
            flag = 0
            for x in l:
                if x == i.name:
                    flag = 1
                    break
            if flag != 1:
                for x in v:
                    if x == i.name:
                        flag = 2
                        break
            if flag == 1:
                color = '#643A71'#path
            elif flag == 0:
                color = '#Fcc201' #
            else:
                color = '#c1c1c1' #visited  grey
            N.add_node(i.name, label=i.name, color=color)

        for v in g.ListofVertices:
            for i in v.Neighbour:
                N.add_edge(v.name, i.name, weight=i.weight, title=i.weight, color='#F42C04')
        N.write_html('basic.html')
        self.widget.load(QUrl.fromLocalFile(os.path.abspath(os.path.join(os.path.dirname(__file__), "basic.html"))))

    def edge_clicked(self, txt, tx2, txt3):
        graphType = str(self.comboBox.currentText())

        if txt == '' or tx2 == '':
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Edge error!")
            msg.setInformativeText("Start or end can't be null")
            msg.setWindowTitle("Error")
            msg.exec_()
            return
        if txt3.isdigit() and graphType=="Undirected Graph":
            t = g.insertEdge_unDirected(txt, tx2, txt3)

        elif txt3.isdigit() and graphType=="Directed Graph":
            t = g.insertEdgeDirected(txt, tx2, txt3)

        elif txt3.isdigit() != True and graphType=="Directed Graph":
            t = g.insertEdgeDirected(txt, tx2)

        else:
            t=g.insertEdge_unDirected(txt, tx2)

        if t == "V1 ERR":
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Edge Error!")
            msg.setInformativeText("Vertex 1 Doesn't Exist")
            msg.setWindowTitle("Error")
            msg.exec_()
            return
        elif t == "V2 ERR":
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Edge Error!")
            msg.setInformativeText("Vertex 2 Doesn't Exist")
            msg.setWindowTitle("Error")
            msg.exec_()
            return
        elif t == "Exist":
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Edge Error!")
            msg.setInformativeText("Edge already exists")
            msg.setWindowTitle("Error")
            msg.exec_()
            return
        self.drawGraph()
        self.lineEdit_10.clear()
        self.lineEdit_11.clear()
        self.lineEdit_12.clear()
        self.lineEdit_16.clear()
        self.lineEdit_17.clear()
        self.lineEdit_18.clear()

    def insert_clicked(self, txt):
        if txt == '':
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Vertex Name!")
            msg.setInformativeText("Vertex name can't be null")
            msg.setWindowTitle("Error")
            msg.exec_()
            return
        t = g.insertVertex(txt)
        if t == -1:
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Vertex Exists!")
            msg.setInformativeText("No Duplicates allowed")
            msg.setWindowTitle("Error")
            msg.exec_()
            return
        self.drawGraph()
        self.lineEdit_9.clear()

    def delete(self, txt):
        if txt == '':
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Which Vertex to delete?")
            msg.setInformativeText("Vertex name can't be null")
            msg.setWindowTitle("Error")
            msg.exec_()
            return
        t = g.deleteVertex(txt)
        if t == "Vertex Doesn't Exist":
            msg = QMessageBox()
            msg.setIcon(QMessageBox.Critical)
            msg.setText("Vertex doesn't Exist!")
            msg.setInformativeText("Enter valid vertex")
            msg.setWindowTitle("Error")
            msg.exec_()
            return
        self.drawGraph()
        self.lineEdit_9.clear()
        self.lineEdit_18.clear()

    def drawGraph(self):
        graphType = str(self.comboBox.currentText())
        bool=True
        if(graphType=="Undirected Graph" ):
            bool=False
        N = Network(height='100%', width='100%', directed=bool )
        for i in g.ListofVertices:
            N.add_node(i.name, label=i.name+"    h= "+str(i.heuristic), color="#Fcc201")
            #print(i.heuristic)
        for v in g.ListofVertices:
            for i in v.Neighbour:
                N.add_edge(v.name, i.name, weight=i.weight, title=i.weight, color='#F42C04')
        N.write_html('basic.html')
        self.widget.load(QUrl.fromLocalFile(os.path.abspath(os.path.join(os.path.dirname(__file__), "basic.html"))))


if __name__ == "__main__":
    import sys

    app = QtWidgets.QApplication(sys.argv)
    Form = QtWidgets.QWidget()
    ui = Ui_Form()
    ui.setupUi(Form)
    Form.show()
    sys.exit(app.exec_())