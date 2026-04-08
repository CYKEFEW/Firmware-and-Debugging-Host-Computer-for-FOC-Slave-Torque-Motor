#!/usr/bin/env python
# -*- coding: utf-8 -*-
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import (QVBoxLayout, QFrame, QSplitter)

from src.gui.configtool.graphicWidget import SimpleFOCGraphicWidget
from src.gui.sharedcomnponets.commandLineInterface import CommandLineWidget
from src.simpleFOCConnector import SimpleFOCDevice


class DeviceInteractionFrame(QFrame):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.layout = QVBoxLayout(self)
        self.setLayout(self.layout)
        self.device = SimpleFOCDevice.getInstance()

        self.graphicWidget = SimpleFOCGraphicWidget(self)
        self.cmdLineTollWidget = CommandLineWidget(self)

        self.cmdLineTollWidget.setMinimumHeight(140)

        self.verticalSplitter = QSplitter(Qt.Vertical)
        self.verticalSplitter.addWidget(self.graphicWidget)
        self.verticalSplitter.addWidget(self.cmdLineTollWidget)
        self.verticalSplitter.setChildrenCollapsible(False)
        self.verticalSplitter.setHandleWidth(10)
        self.verticalSplitter.setStretchFactor(0, 1)
        self.verticalSplitter.setStretchFactor(1, 0)
        self.verticalSplitter.setSizes([760, 180])
        self.device.commProvider.commandDataReceived.connect(
            self.cmdLineTollWidget.publishCommandResponseData)
        self.layout.addWidget(self.verticalSplitter)

        self.setLayout(self.layout)
