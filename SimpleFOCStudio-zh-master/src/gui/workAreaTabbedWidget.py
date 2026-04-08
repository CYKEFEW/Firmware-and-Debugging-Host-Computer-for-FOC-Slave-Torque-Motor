#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json

from PyQt5 import QtCore, QtWidgets

from src.gui.commandlinetool.commandlinetool import CommandLineConsoleTool
from src.gui.configtool.deviceConfigurationTool import DeviceConfigurationTool
from src.gui.configtool.generatedCodeDisplay import GeneratedCodeDisplay
from src.gui.configtool.treeViewConfigTool import TreeViewConfigTool
from src.gui.pidAutoTuneTool import PidAutoTuneTool
from src.simpleFOCConnector import SimpleFOCDevice


class WorkAreaTabbedWidget(QtWidgets.QTabWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setTabsClosable(True)
        self.setMovable(True)
        self.setObjectName('devicesTabWidget')
        self.setUsesScrollButtons(True)
        self.tabBar().setElideMode(QtCore.Qt.ElideRight)
        self.tabBar().setExpanding(False)
        self.tabBar().setDocumentMode(False)

        self.device = SimpleFOCDevice.getInstance()

        self.cmdLineTool = None
        self.configDeviceTool = None
        self.generatedCodeTab = None
        self.pidAutoTuneTab = None
        self.activeToolsList = []

        self.tabCloseRequested.connect(self.removeTabHandler)
        QtCore.QTimer.singleShot(0, self.addDeviceTree)

        self.setStyleSheet("""
            QTabWidget::pane {
                border-top: 1px solid #d6d6d6;
            }
            QTabBar::tab {
                min-height: 30px;
                min-width: 110px;
                max-width: 260px;
                padding: 4px 24px 4px 10px;
                margin-right: 2px;
            }
            QTabBar::close-button {
                subcontrol-origin: padding;
                subcontrol-position: right;
                width: 14px;
                height: 14px;
                margin-left: 6px;
            }
        """)

    def removeTabHandler(self, index):
        widget = self.widget(index)
        if isinstance(widget, CommandLineConsoleTool):
            self.cmdLineTool = None
        if isinstance(widget, (DeviceConfigurationTool, TreeViewConfigTool)):
            self.configDeviceTool = None
        if isinstance(widget, GeneratedCodeDisplay):
            self.generatedCodeTab = None
        if isinstance(widget, PidAutoTuneTool):
            self.pidAutoTuneTab = None

        if 0 <= index < len(self.activeToolsList):
            self.activeToolsList.pop(index)
        self.removeTab(index)

        if (self.configDeviceTool is None and
                self.cmdLineTool is None and
                self.pidAutoTuneTab is None):
            if self.device.isConnected:
                self.device.disConnect()

    def addDeviceForm(self):
        if self.configDeviceTool is None:
            self.configDeviceTool = DeviceConfigurationTool()
            self.activeToolsList.append(self.configDeviceTool)
            self.addTab(self.configDeviceTool,
                        self.configDeviceTool.getTabIcon(), 'Device')
        self.setCurrentWidget(self.configDeviceTool)

    def addDeviceTree(self):
        if self.configDeviceTool is None:
            self.configDeviceTool = TreeViewConfigTool()
            self.activeToolsList.append(self.configDeviceTool)
            self.addTab(self.configDeviceTool,
                        self.configDeviceTool.getTabIcon(), 'Device')
        self.setCurrentWidget(self.configDeviceTool)

    def openDevice(self):
        if self.configDeviceTool is not None:
            self.setCurrentWidget(self.configDeviceTool)
            return

        dlg = QtWidgets.QFileDialog()
        dlg.setFileMode(QtWidgets.QFileDialog.AnyFile)
        if dlg.exec_():
            filenames = dlg.selectedFiles()
            try:
                with open(filenames[0], encoding='utf-8') as json_file:
                    configuration_info = json.load(json_file)
                self.device.configureDevice(configuration_info)
                self.configDeviceTool = TreeViewConfigTool()
                self.device.openedFile = filenames
                self.activeToolsList.append(self.configDeviceTool)
                tab_name = self.configDeviceTool.getTabName() or 'Device'
                self.addTab(self.configDeviceTool,
                            self.configDeviceTool.getTabIcon(), tab_name)
                self.setCurrentWidget(self.configDeviceTool)
            except Exception:
                QtWidgets.QMessageBox.warning(
                    self,
                    'SimpleFOC ConfigTool',
                    '打开配置文件时出错。')

    def saveDevice(self):
        if len(self.activeToolsList) <= 0:
            return
        current_tool = self.activeToolsList[self.currentIndex()]
        if getattr(current_tool.device, 'openedFile', None) is None:
            options = QtWidgets.QFileDialog.Options()
            options |= QtWidgets.QFileDialog.DontUseNativeDialog
            file_name, _ = QtWidgets.QFileDialog.getSaveFileName(
                self,
                '保存电机配置参数',
                '',
                'JSON configuration file (*.json)',
                options=options)
            if file_name:
                self.saveToFile(current_tool.device, file_name)
        else:
            self.saveToFile(current_tool.device, current_tool.device.openedFile)

    def generateCode(self):
        if len(self.activeToolsList) <= 0:
            return
        current_tool = self.activeToolsList[self.currentIndex()]
        self.generatedCodeTab = GeneratedCodeDisplay()
        self.activeToolsList.append(self.generatedCodeTab)
        self.addTab(self.generatedCodeTab,
                    self.generatedCodeTab.getTabIcon(),
                    self.generatedCodeTab.getTabName())
        self.setCurrentWidget(self.generatedCodeTab)

    def saveToFile(self, deviceToSave, file):
        if isinstance(file, list):
            file = file[0]
        with open(file, 'w', encoding='utf-8') as f:
            f.write(json.dumps(deviceToSave.toJSON(), indent=4, sort_keys=True))

    def openConsoleTool(self):
        if self.cmdLineTool is None:
            self.cmdLineTool = CommandLineConsoleTool()
            self.activeToolsList.append(self.cmdLineTool)
            self.addTab(self.cmdLineTool,
                        self.cmdLineTool.getTabIcon(), '命令行交互')
        self.setCurrentWidget(self.cmdLineTool)

    def openPidAutoTuneTool(self):
        if self.pidAutoTuneTab is None:
            self.pidAutoTuneTab = PidAutoTuneTool()
            self.activeToolsList.append(self.pidAutoTuneTab)
            self.addTab(self.pidAutoTuneTab,
                        self.pidAutoTuneTab.getTabIcon(),
                        self.pidAutoTuneTab.getTabName())
        self.setCurrentWidget(self.pidAutoTuneTab)
