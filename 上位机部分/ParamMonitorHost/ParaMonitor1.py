import os
import sys
import copy
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import QTimer, Qt, QRect, QPoint
from PyQt5.QtGui import QStatusTipEvent, QMouseEvent, QPixmap, QPainter, QPen
from PyQt5.QtWidgets import QMessageBox, QApplication, QAction
from PyQt5 import QtGui
from ParamMonitor_ui import Ui_MainWindow
from form_setuart import UartSet
import serial
import serial.tools.list_ports
from PyQt5.QtSerialPort import QSerialPortInfo
from PackUnpack import PackUnpack
from form_resp import FormResp
from form_spo2 import FormSpo2
from form_ecg import FormEcg
from form_savedata import SaveData
from form_playdata import PlayData


class ParamMonitor(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(ParamMonitor, self).__init__()
        self.setupUi(self)
        self.init()
        self.ser = serial.Serial()
        # 定义打包解包类
        self.mPackUnpck = PackUnpack()
        self.mPackAfterUnpackArr = []  # 定义列表，保存解包数据

        # Resp
        self.mRespWaveList = []  # 线性链表，内容为RESP的波形数据
        self.mRespXStep = 0      # Resp横坐标
        self.maxRespLength = self.respWaveLabel.width()
        self.maxRespHeight = self.respWaveLabel.height()
        self.pixmapResp = QPixmap(self.respWaveLabel.width(), self.respWaveLabel.height())
        self.pixmapResp.fill(Qt.white)  # 初始化呼吸画布
        self.respWaveLabel.setPixmap(self.pixmapResp)
        self.painterResp = QPainter(self.pixmapResp)
        # SPO2
        self.mSPO2WaveList = []  # 线性链表，内容为SPO2的波形数据
        self.mSPO2XStep = 0      # SPO2横坐标
        self.maxSPO2Length = self.spo2WaveLabel.width()
        self.maxSPO2Height = self.spo2WaveLabel.height()
        self.pixmapSPO2 = QPixmap(self.spo2WaveLabel.width(), self.spo2WaveLabel.height())
        self.pixmapSPO2.fill(Qt.white)  # 初始化血氧画布
        self.spo2WaveLabel.setPixmap(self.pixmapSPO2)
        self.painterSPO2 = QPainter(self.pixmapSPO2)
        # ECG1
        self.mECG1WaveList = []  # 线性链表，内容为ECG1的波形数据
        self.mECG1XStep = 0      # ECG1横坐标
        self.maxECG1Length = self.ecg1WaveLabel.width()
        self.maxECG1Height = self.ecg1WaveLabel.height()
        self.pixmapECG1 = QPixmap(self.ecg1WaveLabel.width(), self.ecg1WaveLabel.height())
        self.pixmapECG1.fill(Qt.white)  # 初始化ECG1画布
        self.ecg1WaveLabel.setPixmap(self.pixmapECG1)
        self.painterEcg1 = QPainter(self.pixmapECG1)
        # ECG2
        self.mECG2WaveList = []  # 线性链表，内容为ECG2的波形数据
        self.mECG2XStep = 0      # ECG2横坐标
        self.maxECG2Length = self.ecg2WaveLabel.width()
        self.maxECG2Height = self.ecg2WaveLabel.height()
        self.pixmapECG2 = QPixmap(self.ecg2WaveLabel.width(), self.ecg2WaveLabel.height())
        self.pixmapECG2.fill(Qt.white)  # 初始化ECG2画布
        self.ecg2WaveLabel.setPixmap(self.pixmapECG2)
        self.painterEcg2 = QPainter(self.pixmapECG2)

    def init(self):
        # 配置菜单栏，并指定各菜单项单击信号的槽函数
        self.menu1 = QAction(self)
        self.menu1.setText('串口设置')
        self.menubar.addAction(self.menu1)
        self.menu1.triggered.connect(self.slot_serialSet)
        self.menu2 = QAction(self)
        self.menu2.setText('数据存储')
        self.menubar.addAction(self.menu2)
        self.menu2.triggered.connect(self.slot_dataStore)
        self.menu3 = QAction(self)
        self.menu3.setText('演示模式')
        self.menubar.addAction(self.menu3)
        self.menu3.triggered.connect(self.slot_playModel)
        self.menu4 = QAction(self)
        self.menu4.setText('关于')
        self.menubar.addAction(self.menu4)
        self.menu4.triggered.connect(self.slot_about)
        self.menu5 = QAction(self)
        self.menu5.setText('退出')
        self.menubar.addAction(self.menu5)
        self.menu5.triggered.connect(self.slot_quit)
        # 状态栏
        self.statusStr = '串口未打开'
        self.statusBar().showMessage(self.statusStr)
        # 四个画波形区域的边框描黑
        self.ecg1WaveLabel.setStyleSheet("border:1px solid black;")
        self.ecg2WaveLabel.setStyleSheet("border:1px solid black;")
        self.spo2WaveLabel.setStyleSheet("border:1px solid black;")
        self.respWaveLabel.setStyleSheet("border:1px solid black;")
        # 串口接收数据
        self.serialPortTimer = QTimer(self)
        self.serialPortTimer.timeout.connect(self.data_receive)
        # 处理已解包数据的定时器
        self.procDataTimer = QTimer(self)
        self.procDataTimer.timeout.connect(self.data_process)
        # 实现心电图片闪烁的定时器，每1000毫秒交换一次状态
        self.heartShapeTimer = QTimer(self)
        self.heartShapeTimer.timeout.connect(self.heartShapeFlash)
        self.heartShapeTimer.start(1000)
        # 指定组合框的事件监测，实现单击事件响应
        self.respInfoGroupBox.installEventFilter(self)
        self.spo2InfoGroupBox.installEventFilter(self)
        self.ecgInfoGroupBox.installEventFilter(self)
        # 数据存储
        self.saveDataPath = ''  # 存储数据路径
        self.filePath = os.getcwd() + "\savedata.txt"  # 默认存储路径
        self.limit = 0  # 限制每次存储的数据，防止存储文件过大
        # 演示模式
        self.mPlayFlag = False        # 演示模式标志位
        self.mTimerStartFlag = False  # 获取演示数据的定时器开启标志位
        self.mListLoadData = []       # 用于存储从演示文件中读取的数据
        self.mDataAfterPro = []       # 用于存储演示文件中处理后的数据，后面循环获取这个列表的数据
        self.mLoadIndex = 0           # mListLoadData列表的索引值
        self.mLoadDataHead = 0        # mDataAfterPro列表的索引值
        self.playDataPath = ''        # 演示数据路径
        self.proLoadDataTimer = QTimer()  # 处理演示数据的定时器
        self.proLoadDataTimer.timeout.connect(self.proLoadDataThread)  # 关联定时器任务

    # 设置串口开关
    def slot_serialSet(self):
        if self.ser.isOpen():
            self.uartset = UartSet(True)   # 创建串口设置窗口对象，传参用于指定串口状态
        else:
            self.uartset = UartSet(False)  # 创建串口设置窗口对象，传参用于指定串口状态
        self.uartset.serialSignal.connect(self.slot_serial)  # 关联槽函数
        self.uartset.show()  # 打开窗口

    # 配置并打开串口的槽函数
    def slot_serial(self, portNum, baudRate, dataBits, stopBits, parity):
        # 打开串口前若串口处于打开状态，先关闭
        if self.ser.isOpen():
            self.serialPortTimer.stop()
            self.procDataTimer.stop()
            try:
                self.ser.close()
            except:
                pass
            self.statusStr = "串口已关闭"
            self.statusBar().showMessage(self.statusStr)
        else:
            # 若处于演示模式，关闭演示模式，关闭演示模式使用的定时器，清除数据
            if self.mPlayFlag:
                self.mPlayFlag = False
                self.mTimerStartFlag = False
                self.procDataTimer.stop()
                self.proLoadDataTimer.stop()
                self.clearData()
            # 配置串口信息
            self.ser.port = portNum
            self.ser.baudrate = int(baudRate)
            self.ser.bytesize = int(dataBits)
            self.ser.stopbits = int(stopBits)
            self.ser.parity = parity
            # 尝试打开串口，失败时弹出提示框，返回
            try:
                self.ser.open()
            except:
                QMessageBox.critical(self, "Error", "串口打开失败")
                return
            # 成功打开串口时设置状态栏的串口信息为“串口已打开”
            self.statusStr = "串口已打开"
            self.statusBar().showMessage(self.statusStr)
            self.serialPortTimer.start(2)  # 开启处理串口数据的定时器
            self.procDataTimer.start(10)   # 开启处理已解包数据的定时器

    # 通过串口发送字节数据
    def data_send(self, data):
        if self.ser.isOpen():
            data = bytes(data)
            self.ser.write(data)
        else:
            pass

    # 处理串口接收的数据
    def data_receive(self):
        try:
            num = self.ser.inWaiting()  # 获取当前串口缓冲区的数据量
        except:
            self.serialPortTimer.stop()
            self.procDataTimer.stop()
            try:
                self.ser.close()
            except:
                pass
            return None
        if num > 0:
            data = self.ser.read(num)  # 读取当前串口缓冲区的数据
            # 通过for循环遍历data中的数据，直到获取一个完整的数据包时，findPack才为True
            for i in range(0, len(data)):
                findPack = self.mPackUnpck.unpackData(data[i])
                # 解包成功，将数据保存到mPackAfterUnpackArr列表中
                if findPack:
                    temp = self.mPackUnpck.getUnpackRslt()
                    self.mPackAfterUnpackArr.append(copy.deepcopy(temp))
        else:
            pass

    # 处理已解包的数据
    def data_process(self):
        num = len(self.mPackAfterUnpackArr)  # 列表数据长度

        if num > 0:
            for i in range(num):
                packet = self.mPackAfterUnpackArr[i]
                module_id = packet[0]
                second_id = packet[1]

                if module_id == 0x10:  # 波形数据包
                    self.analyzeWaveData(packet)

                elif module_id == 0x11:  # 参数数据包
                    self.analyzeParamData(packet)

                elif module_id == 0x12:  # 状态数据包
                    self.analyzeStatusData(packet)

                # 保存数据
                if len(self.saveDataPath) != 0:
                    if self.limit < 446:
                        with open(self.saveDataPath, 'a') as file:
                            data = packet[:8]  # 取前8个字节
                            file.write(str(data) + "\n")
                            self.limit += 1
                    else:
                        self.saveDataPath = ''
                        self.limit = 0

            # 删除已处理的数据
            del self.mPackAfterUnpackArr[0:num]

        # 绘制波形
        if len(self.mRespWaveList) > 2:
            self.drawRespWave()
        if len(self.mSPO2WaveList) > 2:
            self.drawSPO2Wave()
        if len(self.mECG1WaveList) > 10:
            self.drawECG1Wave()
            self.drawECG2Wave()

    # 处理波形数据包 (0x10)
    def analyzeWaveData(self, data):
        ecg_data = (data[2] << 8) | data[3]
        resp_data = (data[4] << 8) | data[5]
        spo2_data = (data[6] << 8) | data[7]

        self.mECG1WaveList.append(ecg_data)
        self.mECG2WaveList.append(ecg_data)
        self.mRespWaveList.append(resp_data)
        self.mSPO2WaveList.append(spo2_data)

    # 处理参数数据包 (0x11)
    def analyzeParamData(self, data):
        hr = (data[2] << 8) | data[3]
        resp_rate = (data[4] << 8) | data[5]
        spo2_value = (data[6] << 8) | data[7]

        # 心率显示 (0-300)
        if 0 < hr < 300:
            self.heartRateLabel.setText(str(hr))
        else:
            self.heartRateLabel.setText("---")

        # 呼吸率显示 (0-120)
        if 0 < resp_rate < 120:
            self.respRateLabel.setText(str(resp_rate))
        else:
            self.respRateLabel.setText("---")

        # 血氧显示 (0-100)
        if 0 < spo2_value < 100:
            self.labelSPO2Data.setText(str(spo2_value))
        else:
            self.labelSPO2Data.setText("---")

        # 脉率通常等于心率
        if 0 < hr < 320:
            self.labelSPO2PR.setText(str(hr))
        else:
            self.labelSPO2PR.setText("---")

    # 处理状态数据包 (0x12)
    def analyzeStatusData(self, data):
        # ECG导联状态 (data[2] 对应 arrData[0])
        ecg_lead_status = data[2]
        # 每个bit代表一个导联: bit0=LL, bit1=LA, bit2=RA, bit3=V
        leadLL = ecg_lead_status & 0x01
        leadLA = ecg_lead_status & 0x02
        leadRA = ecg_lead_status & 0x04
        leadV = ecg_lead_status & 0x08

        self.leadLLLabel.setStyleSheet("color:red" if leadLL else "color:green")
        self.leadLALabel.setStyleSheet("color:red" if leadLA else "color:green")
        self.leadRALabel.setStyleSheet("color:red" if leadRA else "color:green")
        self.leadVLabel.setStyleSheet("color:red" if leadV else "color:green")

    # # 处理已解包的数据
    # def data_process(self):
    #     num = len(self.mPackAfterUnpackArr)  # 列表数据长度
    #
    #     if num > 0:
    #         for i in range(num):
    #             if self.mPackAfterUnpackArr[i][0] == 0x10:  # 0x10:波形相关的数据包
    #                 # self.analyzeRespData(self.mPackAfterUnpackArr[i])
    #                 # self.analyzeSPO2Data(self.mPackAfterUnpackArr[i])
    #                 self.analyzeECGData(self.mPackAfterUnpackArr[i])
    #
    #             # elif self.mPackAfterUnpackArr[i][0] == 0x11:  # 0x11:参数相关的数据包
    #             #     self.analyzeSPO2Data(self.mPackAfterUnpackArr[i])
    #             # elif self.mPackAfterUnpackArr[i][0] == 0x12:  # 0x12:状态相关的数据包
    #             #     self.analyzeECGData(self.mPackAfterUnpackArr[i])
    #
    #             # 保存数据
    #             if len(self.saveDataPath) != 0:
    #                 # 一次只存储446行数据，防止数据过多，可以根据需求自行更改
    #                 if self.limit < 446:
    #                     with open(self.saveDataPath, 'a') as file:
    #                         data = []
    #                         # 只取mPackAfterUnpackArr[i]的前8个数据，后两个数据无效
    #                         for j in range(0,8):
    #                             data.append(self.mPackAfterUnpackArr[i][j])
    #                         file.write(str(data) + "\n")  # 将数据写入文件，并换行
    #                         self.limit = self.limit + 1
    #                 else:
    #                     self.saveDataPath = ''  # 清空存储路径，关闭数据存储
    #                     self.limit = 0  # 重新计数
    #         # 删掉已处理数据
    #         del self.mPackAfterUnpackArr[0:num]
    #     # 呼吸波形数据点大于2才开始画呼吸波形
    #     if len(self.mRespWaveList) > 2:
    #         self.drawRespWave()
    #     # 血氧波形数据点大于2才开始画血氧波形
    #     if len(self.mSPO2WaveList) > 2:
    #         self.drawSPO2Wave()
    #     # 心电波形数据点大于10才开始画心电波形
    #     if len(self.mECG1WaveList) > 10:
    #         self.drawECG1Wave()
    #         self.drawECG2Wave()

    # # 处理呼吸数据
    # def analyzeRespData(self, data):
    #     if data[1] == 0x02:
    #         for i in range(4, 5):
    #             self.mRespWaveList.append(data[i])
    #     elif data[1] == 0x03:
    #         respRate = data[2] << 8 | data[3]
    #         # respRate的值在0~120才显示
    #         if respRate > 0:
    #             if respRate < 120:
    #                 self.respRateLabel.setText(str(respRate))
    #             else:
    #                 self.respRateLabel.setText("---")
    #
    # # 处理血氧数据
    # def analyzeSPO2Data(self, data):
    #     if data[1] == 0x02:
    #         # for i in range(2, 7):
    #         self.mSPO2WaveList.append((data[6]<<8) | data[7])
    #         fingerLead = (data[7] & 0x80) >> 7
    #         sensorLead = (data[7] & 0x10) >> 4
    #         # if fingerLead == 0x01:
    #         #     self.labelSPO2FingerOff.setStyleSheet("color:red")
    #         #     self.labelSPO2FingerOff.setText("手指脱落")
    #         # else:
    #         #     self.labelSPO2FingerOff.setStyleSheet("color:green")
    #         #     self.labelSPO2FingerOff.setText("手指连接")
    #         # if sensorLead == 0x01:
    #         #     self.labelSPO2PrbOff.setStyleSheet("color:red")
    #         #     self.labelSPO2PrbOff.setText("探头脱落")
    #         # else:
    #         #     self.labelSPO2PrbOff.setStyleSheet("color:green")
    #         #     self.labelSPO2PrbOff.setText("探头连接")
    #
    #     elif data[1] == 0x03:
    #         pulseRate = data[3] << 8 | data[4]
    #         spo2Value = data[5]
    #         # pulseRate的值在0~320才显示
    #         if pulseRate > 0:
    #             if pulseRate < 320:
    #                 self.labelSPO2PR.setText(str(pulseRate))
    #             else:
    #                 self.labelSPO2PR.setText("---")
    #         # spo2Value的值在0~100才显示
    #         if spo2Value > 0:
    #             if spo2Value < 100:
    #                 self.labelSPO2Data.setText(str(spo2Value))
    #             else:
    #                 self.labelSPO2Data.setText("---")
    #
    # # 处理心电数据
    # def analyzeECGData(self, data):
    #     if data[1] == 0x02:
    #         ecg1Data = data[2] << 8 | data[3]
    #         ecg2Data = data[4] << 8 | data[5]
    #         self.mECG1WaveList.append(ecg1Data)  # ECG1波形数据存储到列表1
    #         self.mECG2WaveList.append(ecg2Data)  # ECG2波形数据存储到列表2
    #     elif data[1] == 0x03:
    #         leadLL = data[2] & 0x01
    #         leadLA = data[2] & 0x02
    #         leadRA = data[2] & 0x04
    #         leadV = data[2] & 0x08
    #         if leadLL == 0x01:
    #             self.leadLLLabel.setStyleSheet("color:red")
    #         else:
    #             self.leadLLLabel.setStyleSheet("color:green")
    #         if leadLA == 0x02:
    #             self.leadLALabel.setStyleSheet("color:red")
    #         else:
    #             self.leadLALabel.setStyleSheet("color:green")
    #         if leadRA == 0x04:
    #             self.leadRALabel.setStyleSheet("color:red")
    #         else:
    #             self.leadRALabel.setStyleSheet("color:green")
    #         if leadV == 0x08:
    #             self.leadVLabel.setStyleSheet("color:red")
    #         else:
    #             self.leadVLabel.setStyleSheet("color:green")
    #     elif data[1] == 0x04:
    #         hr = (data[2] << 8) | data[3]
    #         # hr的值在0~300才显示
    #         if hr > 0:
    #             if hr < 300:
    #                 self.heartRateLabel.setText(str(hr))
    #             else:
    #                 self.heartRateLabel.setText("---")

    # 画Resp波形
    def drawRespWave(self):
        iCnt = len(self.mRespWaveList)
        self.painterResp.setBrush(Qt.white)
        self.painterResp.setPen(QPen(Qt.white, 1, Qt.SolidLine))
        if iCnt >= self.maxRespLength - self.mRespXStep:
            # 后半部分刷白
            rct = QRect(self.mRespXStep, 0, self.maxRespLength - self.mRespXStep, self.maxRespHeight)
            self.painterResp.drawRect(rct)
            # 前面部分刷白
            rct = QRect(0, 0, 10 + iCnt - (self.maxRespLength - self.mRespXStep), self.maxRespHeight)
            self.painterResp.drawRect(rct)
        else:
            # 指定部分刷白
            rct = QRect(self.mRespXStep, 0, iCnt + 10, self.maxRespHeight)
            self.painterResp.drawRect(rct)
        # 设置画笔
        self.painterResp.setPen(QPen(Qt.black, 2, Qt.SolidLine))
        # 画图
        for i in range(iCnt - 1):
            point1 = QPoint(self.mRespXStep, self.maxRespHeight - int(self.mRespWaveList[i] / 2))
            point2 = QPoint(self.mRespXStep + 1, self.maxRespHeight - int(self.mRespWaveList[i + 1] / 2))
            self.painterResp.drawLine(point1, point2)
            self.mRespXStep += 1
            if self.mRespXStep >= self.maxRespLength:
                self.mRespXStep = 0
        # 删除iCnt-1个数据，保留最后一个数据，下次画图时起点与现在尾端接上，不会出现断线
        del self.mRespWaveList[0:iCnt - 1]
        # 更新波形
        self.respWaveLabel.setPixmap(self.pixmapResp)

    # 画SPO2波形
    def drawSPO2Wave(self):
        iCnt = len(self.mSPO2WaveList)
        self.painterSPO2.setBrush(Qt.white)
        self.painterSPO2.setPen(QPen(Qt.white, 1, Qt.SolidLine))
        if iCnt >= self.maxSPO2Length - self.mSPO2XStep:
            # 后半部分刷白
            rct = QRect(self.mSPO2XStep, 0, self.maxSPO2Length - self.mSPO2XStep, self.maxSPO2Height)
            self.painterSPO2.drawRect(rct)
            # 前面部分刷白
            rct = QRect(0, 0, 10 + iCnt - (self.maxSPO2Length - self.mSPO2XStep), self.maxSPO2Height)
            self.painterSPO2.drawRect(rct)
        else:
            # 指定部分刷白
            rct = QRect(self.mSPO2XStep, 0, iCnt + 10, self.maxSPO2Height)
            self.painterSPO2.drawRect(rct)
        # 设置画笔
        self.painterSPO2.setPen(QPen(Qt.black, 2, Qt.SolidLine))
        # 画图
        for i in range(iCnt - 1):
            point1 = QPoint(self.mSPO2XStep, self.maxSPO2Height - int(self.mSPO2WaveList[i] / 2))
            point2 = QPoint(self.mSPO2XStep + 1, self.maxSPO2Height - int(self.mSPO2WaveList[i + 1] / 2))
            self.painterSPO2.drawLine(point1, point2)
            self.mSPO2XStep += 1
            if self.mSPO2XStep >= self.maxSPO2Length:
                self.mSPO2XStep = 0
        # 删掉iCnt-1个数据，保留最后一个数据，下次画图时起点与现在尾端接上，不会出现断线
        del self.mSPO2WaveList[0:iCnt - 1]
        # 更新波形
        self.spo2WaveLabel.setPixmap(self.pixmapSPO2)

    # 画ECG1波形
    def drawECG1Wave(self):
        iCnt = len(self.mECG1WaveList)
        self.painterEcg1.setBrush(Qt.white)
        self.painterEcg1.setPen(QPen(Qt.white, 1, Qt.SolidLine))
        if iCnt >= self.maxECG1Length - self.mECG1XStep:
            # 后半部分刷白
            rct = QRect(self.mECG1XStep, 0, self.maxECG1Length - self.mECG1XStep, self.maxECG1Height)
            self.painterEcg1.drawRect(rct)
            # 前面部分刷白
            rct = QRect(0, 0, 10 + iCnt - (self.maxECG1Length - self.mECG1XStep), self.maxECG1Height)
            self.painterEcg1.drawRect(rct)
        else:
            # 指定部分刷白
            rct = QRect(self.mECG1XStep, 0, iCnt + 10, self.maxECG1Height)
            self.painterEcg1.drawRect(rct)
        # 设置画笔
        self.painterEcg1.setPen(QPen(Qt.black, 2, Qt.SolidLine))
        # 画图
        for i in range(iCnt - 1):
            point1 = QPoint(self.mECG1XStep, int(self.maxECG1Height / 2 - (self.mECG1WaveList[i] - 2048) / 15))
            point2 = QPoint(self.mECG1XStep + 1, int(self.maxECG1Height / 2 - (self.mECG1WaveList[i + 1] - 2048) / 15))
            self.painterEcg1.drawLine(point1, point2)
            self.mECG1XStep += 1
            if self.mECG1XStep >= self.maxECG1Length:
                self.mECG1XStep = 0
        # 删除iCnt-1个数据，保留最后一个数据，下次画图时起点与现在尾端接上，不会出现断线
        del self.mECG1WaveList[0:iCnt - 1]
        # 更新波形
        self.ecg1WaveLabel.setPixmap(self.pixmapECG1)

    # 画ECG2波形
    def drawECG2Wave(self):
        iCnt = len(self.mECG2WaveList)
        self.painterEcg2.setBrush(Qt.white)
        self.painterEcg2.setPen(QPen(Qt.white, 1, Qt.SolidLine))
        if iCnt >= self.maxECG2Length - self.mECG2XStep:
            # 后半部分刷白
            rct = QRect(self.mECG2XStep, 0, self.maxECG2Length - self.mECG2XStep, self.maxECG2Height)
            self.painterEcg2.drawRect(rct)
            # 前面部分刷白
            rct = QRect(0, 0, 10 + iCnt - (self.maxECG2Length - self.mECG2XStep), self.maxECG2Height)
            self.painterEcg2.drawRect(rct)
        else:
            # 指定部分刷白
            rct = QRect(self.mECG2XStep, 0, iCnt + 10, self.maxECG2Height)
            self.painterEcg2.drawRect(rct)
        # 设置画笔
        self.painterEcg2.setPen(QPen(Qt.black, 2, Qt.SolidLine))
        # 画图
        for i in range(iCnt - 1):
            point1 = QPoint(self.mECG2XStep, int(self.maxECG2Height / 2 - (self.mECG2WaveList[i] - 2048) / 15))
            point2 = QPoint(self.mECG2XStep + 1, int(self.maxECG2Height / 2 - (self.mECG2WaveList[i + 1] - 2048) / 15))
            self.painterEcg2.drawLine(point1, point2)
            self.mECG2XStep += 1
            if self.mECG2XStep >= self.maxECG2Length:
                self.mECG2XStep = 0
        # 删除iCnt-1个数据，保留最后一个数据，下次画图起点与现在尾端接上，不会出现断线
        del self.mECG2WaveList[0:iCnt - 1]
        # 更新波形
        self.ecg2WaveLabel.setPixmap(self.pixmapECG2)

    # 读取演示文件的数据
    def loadFile(self):
        # 当前路径为空，返回
        if len(self.playDataPath) == 0:
            return
        # 清空mListLoadData列表
        self.mListLoadData = []
        # 以只读（'r'）模式打开当前路径的文件
        with open(self.playDataPath, 'r') as file:
            # 根据文件的行数遍历文件
            for line in file:
                data = []  # 用于存储每一行的数据
                rs = line.replace('\n', '')  # 去除换行符
                rs = rs.replace('[', '')     # 去除'['
                rs = rs.replace(']', '')     # 去除']'
                # 通过识别','的方式，将字符串分割成一个个元素
                data.extend(rs.lstrip().rstrip().split(','))
                # 若当前数据为无效值，不执行后面语句，进入下一轮循环
                if not data:
                    continue
                self.mListLoadData.append(data)
        # 处理演示数据的定时器标志为置为True
        self.mTimerStartFlag = True
        # 开启定时器前先关闭，防止上一次演示模式的影响
        if self.proLoadDataTimer.isActive():
            self.proLoadDataTimer.stop()
        if self.procDataTimer.isActive():
            self.procDataTimer.stop()
        if self.mTimerStartFlag:
            self.proLoadDataTimer.start(2)
            self.procDataTimer.start(10)

    # 处理演示数据
    def proLoadDataThread(self):
        if self.mTimerStartFlag:
            # 遍历mListLoadData列表中的数据
            if len(self.mListLoadData) > self.mLoadIndex:
                listPack = []  # 用于存储mListLoadData每一个索引值的数据
                listPack = self.mListLoadData[self.mLoadIndex]
                # 将listPack中的所有元素转换为十进制数
                for index, item in enumerate(listPack):
                    listPack[index] = int(item, 10)
                # 获取处理后的数据
                self.mPackAfterUnpackArr.append(copy.deepcopy(listPack))
                self.mDataAfterPro.append(copy.deepcopy(listPack))
                self.mLoadIndex = self.mLoadIndex + 1
            else:
                # 循环获取mDataAfterPro中的数据
                self.mPackAfterUnpackArr.append(copy.deepcopy(self.mDataAfterPro[self.mLoadDataHead]))
                self.mLoadDataHead += 1
                if self.mLoadDataHead >= self.mLoadIndex:
                    self.mLoadDataHead = 0

    # 心形闪烁，1000毫秒交换一次状态
    def heartShapeFlash(self):
        self.heartLabel.setVisible(not self.heartLabel.isVisible())

    # 状态栏控件的监听事件
    def event(self, event: QtCore.QEvent) -> bool:
        if event.type() == event.StatusTip:
            if event.tip() == "":
                event = QStatusTipEvent(self.statusStr)
        return super().event(event)

    # 单击各个参数组合框的事件过滤器
    def eventFilter(self, a0: 'QObject', a1: 'QEvent') -> bool:
        if a0 == self.respInfoGroupBox:
            if a1.type() == a1.MouseButtonPress:
                if self.ser.isOpen():
                    self.formResp = FormResp()
                    self.formResp.respSignal.connect(self.slot_resp)
                    self.formResp.show()
                else:
                    QMessageBox.information(None, '消息', '串口未打开', QMessageBox.Ok)
        elif a0 == self.spo2InfoGroupBox:
            if a1.type() == a1.MouseButtonPress:
                if self.ser.isOpen():
                    self.formSpo2 = FormSpo2()
                    self.formSpo2.spo2Signal.connect(self.slot_spo2)
                    self.formSpo2.show()
                else:
                    QMessageBox.information(None, '消息', '串口未打开', QMessageBox.Ok)
        elif a0 == self.ecgInfoGroupBox:
            if a1.type() == a1.MouseButtonPress:
                if self.ser.isOpen():
                    self.formEcg = FormEcg()
                    self.formEcg.ecgSignal.connect(self.slot_ecg)
                    self.formEcg .show()
                else:
                    QMessageBox.information(None, '消息', '串口未打开', QMessageBox.Ok)
        return False

    # 程序退出时的监听事件
    def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
        sys.exit(0)

    # “数据存储”菜单项单击信号的槽函数
    def slot_dataStore(self):
        # 创建数据存储窗口对象
        self.saveData = SaveData(self.filePath)
        # 指定自定义信号的槽函数
        self.saveData.saveDataSignal.connect(self.slot_saveData)
        self.saveData.show()  # 打开窗口

    # “演示模式”菜单项单击信号的槽函数
    def slot_playModel(self):
        # 若串口处于打开状态，关闭串口，关闭对应的定时器，
        if self.ser.isOpen():
            self.serialPortTimer.stop()
            self.procDataTimer.stop()
            try:
                self.ser.close()
            except:
                pass
            # 更新状态栏的串口状态
            self.statusStr = "串口已关闭"
            self.statusBar().showMessage(self.statusStr)
            # 清空所有列表的数据
            self.clearData()
        # 演示模式标志位置为True
        self.mPlayFlag = True
        # 创建数据存储窗口对象
        self.playData = PlayData(self.playDataPath)
        # 指定自定义信号的槽函数
        self.playData.playDataSignal.connect(self.slot_playData)
        self.playData.show()  # 打开窗口

    # “关于”菜单项单击信号的槽函数，用于显示软件信息
    def slot_about(self):
        QMessageBox.information(None, '关于本软件', "LY-M501型医学信号处理平台\n"
                                               "医学信号处理PyQt5软件系统\n"
                                               "版本:V1.0.0\n\n"
                                               "深圳市乐育科技有限公司\n"
                                               "www.leyutek.com", QMessageBox.Ok)

    # “退出”菜单项单击信号的槽函数
    def slot_quit(self):
        app = QApplication.instance()
        app.quit()

    # 呼吸参数设置界面中respSignal信号的槽函数
    def slot_resp(self, data):
        self.data_send(data)

    # 血氧参数设置界面中spo2Signal信号的槽函数
    def slot_spo2(self, data):
        self.data_send(data)

    # 心电参数设置界面中ecgSignal信号的槽函数
    def slot_ecg(self, data):
        self.data_send(data)

    # 获取存储数据路径的槽函数
    def slot_saveData(self, pathStr):
        self.saveDataPath = pathStr
        self.filePath = pathStr

    # 获取演示数据路径的槽函数
    def slot_playData(self, pathStr):
        self.playDataPath = pathStr
        # 演示模式标志位为True，读取文件数据
        if self.mPlayFlag:
            self.loadFile()

    # 用于切换实时模式和演示模式时清空数据
    def clearData(self):
        self.mPackAfterUnpackArr = []
        self.mECG1WaveList = []
        self.mECG2WaveList = []
        self.mSPO2WaveList = []
        self.mRespWaveList = []

