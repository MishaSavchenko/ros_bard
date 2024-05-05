# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'Dialog - untitledmBuFBr.ui'
##
## Created by: Qt User Interface Compiler version 5.15.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide2.QtCore import *
from PySide2.QtGui import *
from PySide2.QtWidgets import *

# import test_rc

class Ui_Dialog(object):
    def setupUi(self, Dialog):
        if not Dialog.objectName():
            Dialog.setObjectName(u"Dialog")
        Dialog.resize(407, 479)
        self.layoutWidget = QWidget(Dialog)
        self.layoutWidget.setObjectName(u"layoutWidget")
        self.layoutWidget.setGeometry(QRect(0, 420, 401, 31))
        self.horizontalLayout = QHBoxLayout(self.layoutWidget)
        self.horizontalLayout.setObjectName(u"horizontalLayout")
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.toolButton_4 = QToolButton(self.layoutWidget)
        self.toolButton_4.setObjectName(u"toolButton_4")
        self.toolButton_4.setArrowType(Qt.LeftArrow)

        self.horizontalLayout.addWidget(self.toolButton_4)

        self.horizontalSlider = QSlider(self.layoutWidget)
        self.horizontalSlider.setObjectName(u"horizontalSlider")
        self.horizontalSlider.setTracking(False)
        self.horizontalSlider.setOrientation(Qt.Horizontal)
        self.horizontalSlider.setTickPosition(QSlider.NoTicks)

        self.horizontalLayout.addWidget(self.horizontalSlider)

        self.toolButton_3 = QToolButton(self.layoutWidget)
        self.toolButton_3.setObjectName(u"toolButton_3")
        self.toolButton_3.setArrowType(Qt.RightArrow)

        self.horizontalLayout.addWidget(self.toolButton_3)

        self.tabWidget = QTabWidget(Dialog)
        self.tabWidget.setObjectName(u"tabWidget")
        self.tabWidget.setGeometry(QRect(0, 40, 401, 381))
        self.tabWidget.setTabShape(QTabWidget.Rounded)
        self.tabWidget.setElideMode(Qt.ElideNone)
        self.tab_6 = QWidget()
        self.tab_6.setObjectName(u"tab_6")
        self.gridLayoutWidget_2 = QWidget(self.tab_6)
        self.gridLayoutWidget_2.setObjectName(u"gridLayoutWidget_2")
        self.gridLayoutWidget_2.setGeometry(QRect(10, 10, 381, 331))
        self.gridLayout_2 = QGridLayout(self.gridLayoutWidget_2)
        self.gridLayout_2.setObjectName(u"gridLayout_2")
        self.gridLayout_2.setContentsMargins(0, 0, 0, 0)
        self.textEdit_4 = QTextEdit(self.gridLayoutWidget_2)
        self.textEdit_4.setObjectName(u"textEdit_4")

        self.gridLayout_2.addWidget(self.textEdit_4, 1, 1, 1, 1)

        self.textEdit_3 = QTextEdit(self.gridLayoutWidget_2)
        self.textEdit_3.setObjectName(u"textEdit_3")

        self.gridLayout_2.addWidget(self.textEdit_3, 1, 0, 1, 1)

        self.lineEdit_3 = QLineEdit(self.gridLayoutWidget_2)
        self.lineEdit_3.setObjectName(u"lineEdit_3")

        self.gridLayout_2.addWidget(self.lineEdit_3, 0, 1, 1, 1)

        self.lineEdit_4 = QLineEdit(self.gridLayoutWidget_2)
        self.lineEdit_4.setObjectName(u"lineEdit_4")
        self.lineEdit_4.setMaximumSize(QSize(126, 16777215))

        self.gridLayout_2.addWidget(self.lineEdit_4, 0, 0, 1, 1)

        self.tabWidget.addTab(self.tab_6, "")
        self.tab = QWidget()
        self.tab.setObjectName(u"tab")
        self.tabWidget.addTab(self.tab, "")
        self.topics = QWidget()
        self.topics.setObjectName(u"topics")
        self.tableWidget = QTableWidget(self.topics)
        if (self.tableWidget.columnCount() < 2):
            self.tableWidget.setColumnCount(2)
        if (self.tableWidget.rowCount() < 1):
            self.tableWidget.setRowCount(1)
        self.tableWidget.setObjectName(u"tableWidget")
        self.tableWidget.setGeometry(QRect(0, 0, 401, 351))
        self.tableWidget.setMidLineWidth(1)
        self.tableWidget.setIconSize(QSize(0, 0))
        self.tableWidget.setGridStyle(Qt.SolidLine)
        self.tableWidget.setRowCount(1)
        self.tableWidget.setColumnCount(2)
        self.tableWidget.horizontalHeader().setVisible(False)
        self.tableWidget.horizontalHeader().setCascadingSectionResizes(False)
        self.tableWidget.horizontalHeader().setProperty("showSortIndicator", False)
        self.tableWidget.horizontalHeader().setStretchLastSection(True)
        self.tableWidget.verticalHeader().setVisible(False)
        self.tableWidget.verticalHeader().setCascadingSectionResizes(False)
        self.tableWidget.verticalHeader().setStretchLastSection(True)
        self.tabWidget.addTab(self.topics, "")
        self.tab_5 = QWidget()
        self.tab_5.setObjectName(u"tab_5")
        self.tabWidget.addTab(self.tab_5, "")

        self.retranslateUi(Dialog)

        self.tabWidget.setCurrentIndex(0)


        QMetaObject.connectSlotsByName(Dialog)
    # setupUi

    def retranslateUi(self, Dialog):
        Dialog.setWindowTitle(QCoreApplication.translate("Dialog", u"Dialog", None))
        self.toolButton_4.setText(QCoreApplication.translate("Dialog", u"...", None))
        self.toolButton_3.setText(QCoreApplication.translate("Dialog", u"...", None))
        self.lineEdit_3.setText(QCoreApplication.translate("Dialog", u"Time Stamp", None))
        self.lineEdit_4.setText(QCoreApplication.translate("Dialog", u"Time Stamp", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_6), QCoreApplication.translate("Dialog", u"Soup", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab), QCoreApplication.translate("Dialog", u"Nodes", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.topics), QCoreApplication.translate("Dialog", u"Topics", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tab_5), QCoreApplication.translate("Dialog", u"Parameters", None))
    # retranslateUi

