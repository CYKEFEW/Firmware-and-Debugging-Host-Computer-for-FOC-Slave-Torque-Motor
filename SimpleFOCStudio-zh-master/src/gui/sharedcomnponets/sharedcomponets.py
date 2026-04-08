#!/usr/bin/env python
# -*- coding: utf-8 -*-
import os

from PyQt5 import QtGui, QtWidgets, QtCore
from serial.tools import list_ports


class GUIToolKit(object):
    ''' This class is used to provide icons for the rest of the application
        hiding the location of the resources
    '''
    RED_COLOR = (255, 92, 92)
    GREEN_COLOR = (57, 217, 138)
    BLUE_COLOR = (91, 141, 236)
    ORANGE_COLOR = (253, 172, 66)
    YELLOW_COLOR = (255,255,51)
    PURPLE_COLOR = (75,0,130)
    MAROON_COLOR = (222,184,135)

    @staticmethod
    def getIconByName(icoName):

        file_index = {
            'add': 'add.png',
            'add_motor': 'add_motor.png',
            'tree': 'tree.png',
            'gen': 'gen.png',
            'home': 'home.png',
            'form': 'form.png',
            'edit': 'edit.png',
            'delete': 'delete.png',
            'statistics': 'statistics.png',
            'reddot': 'reddot.png',
            'orangedot': 'orangedot.png',
            'greendot': 'greendot.png',
            'bluedot': 'bluedot.png',
            'purpledot': 'purpledot.png',
            'yellowdot': 'yellowdot.png',
            'maroondot': 'maroondot.png',
            'send': 'send.png',
            'zoomall': 'zoomall.png',
            'connect': 'connect.png',
            'continue': 'continue.png',
            'alert': 'alert.png',
            'gear': 'gear.png',
            'generalsettings': 'generalsettings.png',
            'open': 'open.png',
            'loop': 'loop.png',
            'save': 'save.png',
            'stop': 'stop.png',
            'restart': 'continue.png',
            'res': 'res.png',
            'sensor': 'sensor.png',
            'start': 'start.png',
            'motor': 'motor.png',
            'pause': 'pause.png',
            'pull': 'pull.png',
            'push': 'push.png',
            'list': 'list.png',
            'disconnect': 'disconnect.png',
            'configure': 'configure.png',
            'pidconfig': 'pidconfig.png',
            'pid': 'pid.png',
            'consoletool': 'consoletool.png',
            'fordward': 'fordward.png',
            'fastbackward': 'fastbackward.png',
            'backward': 'backward.png',
            'stopjogging': 'stopjogging.png',
            'fastfordward': 'fastfordward.png',
            'customcommands':'customcommands.png'
        }
        currentDir = os.path.dirname(__file__)
        icon_path = os.path.join(currentDir, '../resources', file_index[icoName])
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(icon_path), QtGui.QIcon.Normal,
                      QtGui.QIcon.Off)
        return icon


class ConfigQLineEdit(QtWidgets.QLineEdit):
    return_key = 16777220
    updateValue = QtCore.pyqtSignal()
    def __init__(self, parent=None):
        '''Constructor for ToolsWidget'''
        super().__init__(parent)

    def keyPressEvent(self, event):
        if event.key() == self.return_key:
            self.updateValue.emit()
        else:
            super().keyPressEvent(event)

class WorkAreaTabWidget(QtWidgets.QTabWidget):
    def __init__(self, parent=None):
        '''Constructor for ToolsWidget'''
        super().__init__(parent)

    def getTabIcon(self):
        raise NotImplemented

    def getTabName(self):
        raise NotImplemented

class SerialPortComboBox(QtWidgets.QComboBox):
    def __init__(self, parent=None, snifer=None):
        super().__init__(parent)
        self.addItems(self.getAvailableSerialPortNames())

    @staticmethod
    def getAvailablePorts():
        return [port for port in list_ports.comports() if port[2] != 'n/a']

    def getAvailableSerialPortNames(self):
        return [port[0] for port in self.getAvailablePorts()]

    @classmethod
    def getPreferredSerialPortName(cls, current_text=''):
        available_ports = cls.getAvailablePorts()
        available_names = [port[0] for port in available_ports]
        if current_text in available_names:
            return current_text
        if not available_ports:
            return ''

        preferred_keywords = (
            'usb',
            'serial',
            'uart',
            'arduino',
            'silicon labs',
            'cp210',
            'ch340',
            'ch341',
            'ch910',
            'ftdi',
            'esp32',
            'esp',
        )
        for port in available_ports:
            port_desc = ' '.join(str(value).lower() for value in port)
            if any(keyword in port_desc for keyword in preferred_keywords):
                return port[0]
        return available_ports[0][0]

    def selectPreferredPort(self, current_text=''):
        preferred_name = self.getPreferredSerialPortName(current_text)
        if preferred_name:
            self.setCurrentText(preferred_name)
        return preferred_name

    def showPopup(self):
        selectedItem = self.currentText()
        super().clear()
        availableSerialPortNames = self.getAvailableSerialPortNames()
        self.addItems(availableSerialPortNames)
        preferred_item = self.getPreferredSerialPortName(selectedItem)
        if preferred_item:
            self.setCurrentText(preferred_item)
        super().showPopup()
