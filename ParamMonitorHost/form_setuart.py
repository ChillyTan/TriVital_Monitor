# *_* coding : UTF-8 *_*
# 开发团队 ：乐育科技
# 开发人员 ：zcq
# 开发时间 ：2022/6/14 18:17
# 文件名称 ：form_setuart.PY 
# 开发工具 ：PyCharm
from PyQt5 import QtWidgets, QtGui
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QApplication
from form_setuart_ui import Ui_FormSetUART
import serial
# 获取所有串口信息
import serial.tools.list_ports
from PyQt5.QtSerialPort import QSerialPortInfo


class UartSet(QtWidgets.QWidget, Ui_FormSetUART):
    # 自定义信号
    serialSignal = pyqtSignal(str, str, str, str, str)

    def __init__(self, flag):
        super(UartSet, self).__init__()
        self.setupUi(self)
        # 设置串口打开标志图片
        if flag:
            self.uartStsLabel.setPixmap(QtGui.QPixmap(":/new/prefix1/image/open.png"))
            self.openUARTButton.setText("关闭串口")
        else:
            self.uartStsLabel.setPixmap(QtGui.QPixmap(":/new/prefix1/image/close.png"))
            self.openUARTButton.setText("打开串口")
        self.init()
        self.serial_search()

    def init(self):
        # 关联槽函数
        self.openUARTButton.clicked.connect(self.openUart)

    # 搜索串口号
    def serial_search(self):
        port_lsit = QSerialPortInfo.availablePorts()  # 获取有效的串口号
        # 将有效的串口号添加到串口选择下拉列表中
        if len(port_lsit) >= 1:
            self.uartNumComboBox.clear()
            for i in port_lsit:
                self.uartNumComboBox.addItem(i.portName())

    # “打开串口”按钮单击信号的槽函数
    def openUart(self):
        self.serialSignal.emit(self.uartNumComboBox.currentText(),
                               self.baudRateComboBox.currentText(),
                               self.dataBitsComboBox.currentText(),
                               self.stopBitsComboBox.currentText(),
                               self.parityComboBox.currentText())
        self.close() 
 
