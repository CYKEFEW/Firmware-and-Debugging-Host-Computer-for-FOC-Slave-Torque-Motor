#!/usr/bin/env python
# -*- coding: utf-8 -*-
import logging
import os
import sys

os.environ.setdefault('QT_LOGGING_RULES', 'qt.qpa.fonts=false')

""" This module contains ans script to start the SimpleFOC ConfigTool, a GIU
    application ta monitor, tune and configure BLDC motor controllers based on
    SimpleFOC library.
"""
from PyQt5 import QtGui, QtWidgets

from src.gui.mainWindow import UserInteractionMainWindow


def _pick_app_font():
    families = set(QtGui.QFontDatabase().families())
    for family in [
        'Microsoft YaHei UI',
        'Microsoft YaHei',
        'Segoe UI',
        'SimSun',
        'Arial',
    ]:
        if family in families:
            return QtGui.QFont(family, 10)
    return QtGui.QFont()

if __name__ == '__main__':
    try:
        logging.basicConfig(
            filename='.SimpleFOCConfigTool.log',
            filemode='w',
            format='%(name)s - %(levelname)s - %(message)s')
        app = QtWidgets.QApplication(sys.argv)
        app.setStyle('Fusion')
        app.setFont(_pick_app_font())
        mainWindow = QtWidgets.QMainWindow()
        userInteractionMainWindow = UserInteractionMainWindow()
        userInteractionMainWindow.setupUi(mainWindow)
        mainWindow.show()
        sys.exit(app.exec_())
    except Exception as exception:
        logging.error(exception, exc_info=True)
