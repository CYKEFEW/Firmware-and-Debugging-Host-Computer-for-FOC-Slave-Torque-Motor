# !/usr/bin/env python
# -*- coding: utf-8 -*-
from PyQt5 import QtWidgets

from src.gui.sharedcomnponets.sharedcomponets import GUIToolKit


class SimpleFOCConfigToolBar(QtWidgets.QToolBar):
    def __init__(self, main_window, devicesTabedWidget, parent=None):
        super().__init__(parent)

        self.addDeviceAction = QtWidgets.QToolButton(main_window)
        self.addDeviceAction.setIcon(GUIToolKit.getIconByName('add_motor'))
        self.addDeviceAction.setObjectName('addDeviceAction')
        self.addDeviceAction.setPopupMode(QtWidgets.QToolButton.InstantPopup)

        self.addDeviceMenu = QtWidgets.QMenu(self.addDeviceAction)
        self.addDeviceTreeView = QtWidgets.QAction("树形视图", self.addDeviceMenu)
        self.addDeviceTreeView.setIcon(GUIToolKit.getIconByName('tree'))
        self.addDeviceTreeView.triggered.connect(devicesTabedWidget.addDeviceTree)
        self.addDeviceFormView = QtWidgets.QAction("表单视图", self.addDeviceMenu)
        self.addDeviceFormView.setIcon(GUIToolKit.getIconByName('form'))
        self.addDeviceFormView.triggered.connect(devicesTabedWidget.addDeviceForm)

        self.addDeviceMenu.addAction(self.addDeviceTreeView)
        self.addDeviceMenu.addAction(self.addDeviceFormView)
        self.addDeviceAction.setMenu(self.addDeviceMenu)
        self.addWidget(self.addDeviceAction)

        self.openDeviceAction = QtWidgets.QAction(main_window)
        self.openDeviceAction.setIcon(GUIToolKit.getIconByName('open'))
        self.openDeviceAction.setObjectName('openDeviceAction')
        self.openDeviceAction.triggered.connect(devicesTabedWidget.openDevice)
        self.addAction(self.openDeviceAction)

        self.saveDeviceAction = QtWidgets.QAction(main_window)
        self.saveDeviceAction.setIcon(GUIToolKit.getIconByName('save'))
        self.saveDeviceAction.setObjectName('saveDeviceAction')
        self.saveDeviceAction.triggered.connect(devicesTabedWidget.saveDevice)
        self.addAction(self.saveDeviceAction)

        self.generateCodeAction = QtWidgets.QAction(main_window)
        self.generateCodeAction.setIcon(GUIToolKit.getIconByName('gen'))
        self.generateCodeAction.setObjectName('genertecode')
        self.generateCodeAction.triggered.connect(devicesTabedWidget.generateCode)
        self.addAction(self.generateCodeAction)

        self.pidAutoTuneAction = QtWidgets.QAction(main_window)
        self.pidAutoTuneAction.setIcon(GUIToolKit.getIconByName('pid'))
        self.pidAutoTuneAction.setToolTip('PID自动测量')
        self.pidAutoTuneAction.setObjectName('pidautotune')
        self.pidAutoTuneAction.triggered.connect(devicesTabedWidget.openPidAutoTuneTool)
        self.addAction(self.pidAutoTuneAction)

        self.addSeparator()

        self.openConsoleToolAction = QtWidgets.QAction(main_window)
        self.openConsoleToolAction.setIcon(GUIToolKit.getIconByName('consoletool'))
        self.openConsoleToolAction.setToolTip('命令行交互')
        self.openConsoleToolAction.setObjectName('openconsoletool')
        self.openConsoleToolAction.triggered.connect(devicesTabedWidget.openConsoleTool)
        self.addAction(self.openConsoleToolAction)

        self.addSeparator()
