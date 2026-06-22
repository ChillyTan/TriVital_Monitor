from PyQt5 import QtWidgets, QtGui
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtSerialPort import QSerialPortInfo
from qfluentwidgets import ComboBox, PushButton

from form_setuart_ui import Ui_FormSetUART
from ui_theme import bold_font, uart_dialog_style


class UartSet(QtWidgets.QWidget, Ui_FormSetUART):
    serialSignal = pyqtSignal(str, str, str, str, str)

    def __init__(self, opened):
        super(UartSet, self).__init__()
        self.setupUi(self)
        self.apply_fluent_controls()
        self.port_names = []
        self.apply_fonts()
        self.apply_style()
        self.openUARTButton.clicked.connect(self.openUart)
        self.set_open_state(opened)
        self.serial_search()

    def apply_fluent_controls(self):
        combo_names = (
            "uartNumComboBox",
            "baudRateComboBox",
            "dataBitsComboBox",
            "stopBitsComboBox",
            "parityComboBox",
        )
        for name in combo_names:
            old_combo = getattr(self, name)
            new_combo = ComboBox(self)
            new_combo.setGeometry(old_combo.geometry())
            new_combo.setObjectName(old_combo.objectName())
            new_combo.setFont(old_combo.font())
            for index in range(old_combo.count()):
                new_combo.addItem(old_combo.itemText(index))
            new_combo.setCurrentIndex(old_combo.currentIndex())
            old_combo.hide()
            setattr(self, name, new_combo)

        old_button = self.openUARTButton
        self.openUARTButton = PushButton(old_button.text(), self)
        self.openUARTButton.setGeometry(old_button.geometry())
        self.openUARTButton.setObjectName(old_button.objectName())
        self.openUARTButton.setFont(old_button.font())
        old_button.hide()

    def apply_fonts(self):
        zh_font = bold_font("SimHei", 12)
        mono_font = bold_font("DejaVu Sans Mono", 12)

        for widget in (
            self.uartNumLabel,
            self.baudRateLabel,
            self.dataBitsLabel,
            self.stopBitsLabel,
            self.parityLabel,
            self.openUARTButton,
        ):
            widget.setFont(zh_font)

        for widget in (
            self.uartNumComboBox,
            self.baudRateComboBox,
            self.dataBitsComboBox,
            self.stopBitsComboBox,
            self.parityComboBox,
        ):
            widget.setFont(mono_font)

    def apply_style(self):
        self.setWindowTitle("串口设置")
        self.setStyleSheet(uart_dialog_style())

    def set_open_state(self, opened):
        icon_name = "open.png" if opened else "close.png"
        button_text = "关闭串口" if opened else "打开串口"
        self.uartStsLabel.setPixmap(QtGui.QPixmap(f":/new/prefix1/image/{icon_name}"))
        self.openUARTButton.setText(button_text)
        self.openUARTButton.setEnabled(True)

    def serial_search(self):
        ports = QSerialPortInfo.availablePorts()
        self.uartNumComboBox.clear()
        self.port_names = []

        if not ports:
            self.uartNumComboBox.addItem("未检测到串口")
            self.openUARTButton.setEnabled(False)
            return

        for port in ports:
            port_name = port.portName()
            description = port.description()
            display_name = port_name if not description else f"{port_name} - {description}"
            self.port_names.append(port_name)
            self.uartNumComboBox.addItem(display_name)

        self.openUARTButton.setEnabled(True)

    def openUart(self):
        if not self.port_names:
            QtWidgets.QMessageBox.warning(self, "串口设置", "未检测到可用串口")
            return

        index = self.uartNumComboBox.currentIndex()
        port_name = self.port_names[index] if 0 <= index < len(self.port_names) else self.uartNumComboBox.currentText()
        self.serialSignal.emit(
            port_name,
            self.baudRateComboBox.currentText(),
            self.dataBitsComboBox.currentText(),
            self.stopBitsComboBox.currentText(),
            self.parityComboBox.currentText(),
        )
        self.close()
