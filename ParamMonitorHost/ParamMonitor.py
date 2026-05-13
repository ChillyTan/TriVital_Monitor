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
        self.mPackUnpck = PackUnpack()
        self.mPackAfterUnpackArr = []
        self.mRespWaveList = []
        self.mRespXStep = 0
        self.maxRespLength = self.respWaveLabel.width()
        self.maxRespHeight = self.respWaveLabel.height()
        self.pixmapResp = QPixmap(self.respWaveLabel.width(), self.respWaveLabel.height())
        self.pixmapResp.fill(Qt.white)
        self.respWaveLabel.setPixmap(self.pixmapResp)
        self.painterResp = QPainter(self.pixmapResp)
        self.mSPO2WaveList = []
        self.mSPO2XStep = 0
        self.maxSPO2Length = self.spo2WaveLabel.width()
        self.maxSPO2Height = self.spo2WaveLabel.height()
        self.pixmapSPO2 = QPixmap(self.spo2WaveLabel.width(), self.spo2WaveLabel.height())
        self.pixmapSPO2.fill(Qt.white)
        self.spo2WaveLabel.setPixmap(self.pixmapSPO2)
        self.painterSPO2 = QPainter(self.pixmapSPO2)
        self.mECG1WaveList = []
        self.mECG1XStep = 0
        self.maxECG1Length = self.ecg1WaveLabel.width()
        self.maxECG1Height = self.ecg1WaveLabel.height()
        self.pixmapECG1 = QPixmap(self.ecg1WaveLabel.width(), self.ecg1WaveLabel.height())
        self.pixmapECG1.fill(Qt.white)
        self.ecg1WaveLabel.setPixmap(self.pixmapECG1)
        self.painterEcg1 = QPainter(self.pixmapECG1)
        self.convert_signed_16bit = lambda high_byte, low_byte: (high_byte << 8 | low_byte) if (high_byte << 8 | low_byte) < 32768 else (high_byte << 8 | low_byte) - 65536
        self.adaptive_scale_enabled = True
        self.ecg_min_val, self.ecg_max_val = float('inf'), float('-inf')
        self.resp_min_val, self.resp_max_val = float('inf'), float('-inf')
        self.spo2_min_val, self.spo2_max_val = float('inf'), float('-inf')
        self.scale_update_counter = 0
        self.scale_update_interval = 50
        self.sliding_window_size = 200
        self.ecg_sliding_buffer = []
        self.resp_sliding_buffer = []
        self.spo2_sliding_buffer = []
        self.sync_error_count = 0
        self.sync_error_threshold = 50

    def init(self):
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
        self.statusStr = '串口未打开'
        self.statusBar().showMessage(self.statusStr)
        self.ecg1WaveLabel.setStyleSheet("border:1px solid black;")
        self.spo2WaveLabel.setStyleSheet("border:1px solid black;")
        self.respWaveLabel.setStyleSheet("border:1px solid black;")
        self.serialPortTimer = QTimer(self)
        self.serialPortTimer.timeout.connect(self.data_receive)
        self.procDataTimer = QTimer(self)
        self.procDataTimer.timeout.connect(self.data_process)
        self.heartShapeTimer = QTimer(self)
        self.heartShapeTimer.timeout.connect(self.heartShapeFlash)
        self.heartShapeTimer.start(1000)
        self.respInfoGroupBox.installEventFilter(self)
        self.spo2InfoGroupBox.installEventFilter(self)
        self.ecgInfoGroupBox.installEventFilter(self)
        self.saveDataPath = ''
        self.filePath = os.path.join(os.getcwd(), "savedata.txt")
        self.limit = 0
        self.mPlayFlag = False
        self.mTimerStartFlag = False
        self.mListLoadData = []
        self.mDataAfterPro = []
        self.mLoadIndex = 0
        self.mLoadDataHead = 0
        self.playDataPath = ''
        self.proLoadDataTimer = QTimer()
        self.proLoadDataTimer.timeout.connect(self.proLoadDataThread)

    def slot_serialSet(self):
        if self.ser.isOpen():
            self.uartset = UartSet(True)
        else:
            self.uartset = UartSet(False)
        self.uartset.serialSignal.connect(self.slot_serial)
        self.uartset.show()

    def slot_serial(self, portNum, baudRate, dataBits, stopBits, parity):
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
            if self.mPlayFlag:
                self.mPlayFlag = False
                self.mTimerStartFlag = False
                self.procDataTimer.stop()
                self.proLoadDataTimer.stop()
                self.clearData()
            self.ser.port = portNum
            self.ser.baudrate = int(baudRate)
            self.ser.bytesize = int(dataBits)
            self.ser.stopbits = int(stopBits)
            self.ser.parity = parity
            try:
                self.ser.open()
            except:
                QMessageBox.critical(self, "Error", "串口打开失败")
                return
            self.statusStr = "串口已打开"
            self.statusBar().showMessage(self.statusStr)
            self.serialPortTimer.start(2)
            self.procDataTimer.start(10)

    def data_send(self, data):
        if self.ser.isOpen():
            data = bytes(data)
            self.ser.write(data)
        else:
            pass

    def data_receive(self):
        try:
            num = self.ser.inWaiting()
        except:
            self.serialPortTimer.stop()
            self.procDataTimer.stop()
            try:
                self.ser.close()
            except:
                pass
            return None
        if num > 0:
            data = self.ser.read(num)
            for i in range(0, len(data)):
                byte = data[i]
                if self.mPackUnpck.sGotPackId:
                    if byte < 0x80 and self.mPackUnpck.sPackLen < 10:
                        self.sync_error_count += 1
                        self.mPackUnpck.sGotPackId = False
                        self.mPackUnpck.sPackLen = 0
                        self.mPackUnpck.sRestByteNum = 0
                    elif byte >= 0x80 and self.mPackUnpck.sPackLen > 0 and self.mPackUnpck.sPackLen < 10:
                        pass
                else:
                    if byte < 0x80:
                        if hasattr(self, 'sync_error_count') and self.sync_error_count > self.sync_error_threshold:
                            self.reset_packet_sync()
                            self.sync_error_count = 0
                findPack = self.mPackUnpck.unpackData(byte)
                if findPack:
                    self.sync_error_count = 0
                    temp = self.mPackUnpck.getUnpackRslt()
                    self.mPackAfterUnpackArr.append(copy.deepcopy(temp))
                elif byte < 0x80 and not self.mPackUnpck.sGotPackId:
                    pass
        else:
            if hasattr(self, 'sync_error_count') and self.sync_error_count > self.sync_error_threshold:
                self.reset_packet_sync()
                self.sync_error_count = 0

    def data_process(self):
        num = len(self.mPackAfterUnpackArr)
        if num > 0:
            for i in range(num):
                packet = self.mPackAfterUnpackArr[i]
                module_id = packet[0]

                if module_id == 0x10:
                    self.analyzeWaveData(packet)
                elif module_id == 0x11:
                    self.analyzeParamData(packet)
                elif module_id == 0x12:
                    self.analyzeStatusData(packet)
                if len(self.saveDataPath) != 0:
                    if self.limit < 446:
                        with open(self.saveDataPath, 'a') as file:
                            data = []
                            for j in range(0, min(len(packet), 8)):
                                data.append(packet[j])
                            file.write(str(data) + "\n")
                            self.limit = self.limit + 1
                    else:
                        self.saveDataPath = ''
                        self.limit = 0
            del self.mPackAfterUnpackArr[0:num]
        if len(self.mRespWaveList) > 2:
            self.drawRespWave()
        if len(self.mSPO2WaveList) > 2:
            self.drawSPO2Wave()
        if len(self.mECG1WaveList) > 10:
            self.drawECG1Wave()

    def analyzeWaveData(self, data):
        ecg_data = self.convert_signed_16bit(data[2], data[3])
        resp_data = self.convert_signed_16bit(data[4], data[5])
        spo2_data = self.convert_signed_16bit(data[6], data[7])
        if self.adaptive_scale_enabled:
            self.ecg_sliding_buffer.append(ecg_data)
            self.resp_sliding_buffer.append(resp_data)
            self.spo2_sliding_buffer.append(spo2_data)
            if len(self.ecg_sliding_buffer) > self.sliding_window_size:
                self.ecg_sliding_buffer.pop(0)
            if len(self.resp_sliding_buffer) > self.sliding_window_size:
                self.resp_sliding_buffer.pop(0)
            if len(self.spo2_sliding_buffer) > self.sliding_window_size:
                self.spo2_sliding_buffer.pop(0)
            if self.ecg_sliding_buffer:
                self.ecg_min_val = min(self.ecg_sliding_buffer)
                self.ecg_max_val = max(self.ecg_sliding_buffer)
            if self.resp_sliding_buffer:
                self.resp_min_val = min(self.resp_sliding_buffer)
                self.resp_max_val = max(self.resp_sliding_buffer)
            if self.spo2_sliding_buffer:
                self.spo2_min_val = min(self.spo2_sliding_buffer)
                self.spo2_max_val = max(self.spo2_sliding_buffer)
            self.scale_update_counter += 1
        self.mECG1WaveList.append(ecg_data)
        self.mRespWaveList.append(resp_data)
        self.mSPO2WaveList.append(spo2_data)

    def analyzeParamData(self, data):
        hr = (data[2] << 8) | data[3]
        resp_rate = (data[4] << 8) | data[5]
        spo2_value = (data[6] << 8) | data[7]

        if 0 < hr < 300:
            self.heartRateLabel.setText(str(hr))
        else:
            self.heartRateLabel.setText("---")
        if 0 < resp_rate < 120:
            self.respRateLabel.setText(str(resp_rate))
        else:
            self.respRateLabel.setText("---")
        if 0 <= spo2_value <= 100:
            self.labelSPO2Data.setText(str(spo2_value))
        else:
            self.labelSPO2Data.setText("---")
        # ========================================================

    def analyzeStatusData(self, data):
        ecg_lead_status = data[2]
        leadLL = ecg_lead_status & 0x01
        leadLA = ecg_lead_status & 0x02
        leadRA = ecg_lead_status & 0x04
        leadV = ecg_lead_status & 0x08
        self.leadLLLabel.setStyleSheet("color:red" if leadLL else "color:green")
        self.leadLALabel.setStyleSheet("color:red" if leadLA else "color:green")
        self.leadRALabel.setStyleSheet("color:red" if leadRA else "color:green")
        self.leadVLabel.setStyleSheet("color:red" if leadV else "color:green")

    def drawRespWave(self):
        iCnt = len(self.mRespWaveList)
        if iCnt == 0:
            return
        if self.adaptive_scale_enabled and self.scale_update_counter >= self.scale_update_interval:
            if self.resp_max_val == self.resp_min_val:
                self.resp_min_val -= 100
                self.resp_max_val += 100
        self.painterResp.setBrush(Qt.white)
        self.painterResp.setPen(QPen(Qt.white, 1, Qt.SolidLine))
        if iCnt >= self.maxRespLength - self.mRespXStep:
            rct = QRect(self.mRespXStep, 0, self.maxRespLength - self.mRespXStep, self.maxRespHeight)
            self.painterResp.drawRect(rct)
            rct = QRect(0, 0, 10 + iCnt - (self.maxRespLength - self.mRespXStep), self.maxRespHeight)
            self.painterResp.drawRect(rct)
        else:
            rct = QRect(self.mRespXStep, 0, iCnt + 10, self.maxRespHeight)
            self.painterResp.drawRect(rct)
        self.painterResp.setPen(QPen(Qt.black, 2, Qt.SolidLine))
        for i in range(iCnt - 1):
            if self.adaptive_scale_enabled and self.resp_max_val != self.resp_min_val:
                data_range = self.resp_max_val - self.resp_min_val
                buffer = data_range * 0.1
                adjusted_min = self.resp_min_val - buffer
                adjusted_max = self.resp_max_val + buffer
                adjusted_range = adjusted_max - adjusted_min
                y1 = int(((adjusted_max - self.mRespWaveList[i]) / adjusted_range) * self.maxRespHeight)
                y2 = int(((adjusted_max - self.mRespWaveList[i + 1]) / adjusted_range) * self.maxRespHeight)
            else:
                y1 = int(((32767 - self.mRespWaveList[i]) / 65535.0) * self.maxRespHeight)
                y2 = int(((32767 - self.mRespWaveList[i + 1]) / 65535.0) * self.maxRespHeight)
            y1 = max(0, min(y1, self.maxRespHeight))
            y2 = max(0, min(y2, self.maxRespHeight))
            point1 = QPoint(self.mRespXStep, y1)
            point2 = QPoint(self.mRespXStep + 1, y2)
            self.painterResp.drawLine(point1, point2)
            self.mRespXStep += 1
            if self.mRespXStep >= self.maxRespLength:
                self.mRespXStep = 0
        del self.mRespWaveList[0:iCnt - 1]
        self.respWaveLabel.setPixmap(self.pixmapResp)

    def reset_packet_sync(self):
        self.mPackUnpck.sGotPackId = False
        self.mPackUnpck.sPackLen = 0
        self.mPackUnpck.sRestByteNum = 0
        self.mPackUnpck.mBufList = [0] * 10

    def drawSPO2Wave(self):
        iCnt = len(self.mSPO2WaveList)
        if iCnt == 0:
            return
        if self.adaptive_scale_enabled and self.scale_update_counter >= self.scale_update_interval:
            if self.spo2_max_val == self.spo2_min_val:
                self.spo2_min_val -= 100
                self.spo2_max_val += 100
        self.painterSPO2.setBrush(Qt.white)
        self.painterSPO2.setPen(QPen(Qt.white, 1, Qt.SolidLine))
        if iCnt >= self.maxSPO2Length - self.mSPO2XStep:
            rct = QRect(self.mSPO2XStep, 0, self.maxSPO2Length - self.mSPO2XStep, self.maxSPO2Height)
            self.painterSPO2.drawRect(rct)
            rct = QRect(0, 0, 10 + iCnt - (self.maxSPO2Length - self.mSPO2XStep), self.maxSPO2Height)
            self.painterSPO2.drawRect(rct)
        else:
            rct = QRect(self.mSPO2XStep, 0, iCnt + 10, self.maxSPO2Height)
            self.painterSPO2.drawRect(rct)
        self.painterSPO2.setPen(QPen(Qt.black, 2, Qt.SolidLine))
        for i in range(iCnt - 1):
            if self.adaptive_scale_enabled and self.spo2_max_val != self.spo2_min_val:
                data_range = self.spo2_max_val - self.spo2_min_val
                buffer = data_range * 0.1
                adjusted_min = self.spo2_min_val - buffer
                adjusted_max = self.spo2_max_val + buffer
                adjusted_range = adjusted_max - adjusted_min
                y1 = int(((adjusted_max - self.mSPO2WaveList[i]) / adjusted_range) * self.maxSPO2Height)
                y2 = int(((adjusted_max - self.mSPO2WaveList[i + 1]) / adjusted_range) * self.maxSPO2Height)
            else:
                y1 = int(((32767 - self.mSPO2WaveList[i]) / 65535.0) * self.maxSPO2Height)
                y2 = int(((32767 - self.mSPO2WaveList[i + 1]) / 65535.0) * self.maxSPO2Height)
            y1 = max(0, min(y1, self.maxSPO2Height))
            y2 = max(0, min(y2, self.maxSPO2Height))
            point1 = QPoint(self.mSPO2XStep, y1)
            point2 = QPoint(self.mSPO2XStep + 1, y2)
            self.painterSPO2.drawLine(point1, point2)
            self.mSPO2XStep += 1
            if self.mSPO2XStep >= self.maxSPO2Length:
                self.mSPO2XStep = 0
        del self.mSPO2WaveList[0:iCnt - 1]
        self.spo2WaveLabel.setPixmap(self.pixmapSPO2)

    def drawECG1Wave(self):
        iCnt = len(self.mECG1WaveList)
        if iCnt == 0:
            return
        if self.adaptive_scale_enabled and self.scale_update_counter >= self.scale_update_interval:
            if self.ecg_max_val == self.ecg_min_val:
                self.ecg_min_val -= 100
                self.ecg_max_val += 100
            self.scale_update_counter = 0
        self.painterEcg1.setBrush(Qt.white)
        self.painterEcg1.setPen(QPen(Qt.white, 1, Qt.SolidLine))
        if iCnt >= self.maxECG1Length - self.mECG1XStep:
            rct = QRect(self.mECG1XStep, 0, self.maxECG1Length - self.mECG1XStep, self.maxECG1Height)
            self.painterEcg1.drawRect(rct)
            rct = QRect(0, 0, 10 + iCnt - (self.maxECG1Length - self.mECG1XStep), self.maxECG1Height)
            self.painterEcg1.drawRect(rct)
        else:
            rct = QRect(self.mECG1XStep, 0, iCnt + 10, self.maxECG1Height)
            self.painterEcg1.drawRect(rct)
        self.painterEcg1.setPen(QPen(Qt.black, 2, Qt.SolidLine))
        for i in range(iCnt - 1):
            if self.adaptive_scale_enabled and self.ecg_max_val != self.ecg_min_val:
                data_range = self.ecg_max_val - self.ecg_min_val
                buffer = data_range * 0.1
                adjusted_min = self.ecg_min_val - buffer
                adjusted_max = self.ecg_max_val + buffer
                adjusted_range = adjusted_max - adjusted_min
                y1 = int(((adjusted_max - self.mECG1WaveList[i]) / adjusted_range) * self.maxECG1Height)
                y2 = int(((adjusted_max - self.mECG1WaveList[i + 1]) / adjusted_range) * self.maxECG1Height)
            else:
                y1 = int(((32767 - self.mECG1WaveList[i]) / 65535.0) * self.maxECG1Height)
                y2 = int(((32767 - self.mECG1WaveList[i + 1]) / 65535.0) * self.maxECG1Height)
            y1 = max(0, min(y1, self.maxECG1Height))
            y2 = max(0, min(y2, self.maxECG1Height))
            point1 = QPoint(self.mECG1XStep, y1)
            point2 = QPoint(self.mECG1XStep + 1, y2)
            self.painterEcg1.drawLine(point1, point2)
            self.mECG1XStep += 1
            if self.mECG1XStep >= self.maxECG1Length:
                self.mECG1XStep = 0
        del self.mECG1WaveList[0:iCnt - 1]
        self.ecg1WaveLabel.setPixmap(self.pixmapECG1)

    def loadFile(self):
        if len(self.playDataPath) == 0:
            return
        self.mListLoadData = []
        with open(self.playDataPath, 'r') as file:
            for line in file:
                data = []
                rs = line.replace('\n', '')
                rs = rs.replace('[', '')
                rs = rs.replace(']', '')
                data.extend(rs.lstrip().rstrip().split(','))
                if not data:
                    continue
                self.mListLoadData.append(data)
        self.mTimerStartFlag = True
        if self.proLoadDataTimer.isActive():
            self.proLoadDataTimer.stop()
        if self.procDataTimer.isActive():
            self.procDataTimer.stop()
        if self.mTimerStartFlag:
            self.proLoadDataTimer.start(2)
            self.procDataTimer.start(10)

    def proLoadDataThread(self):
        if self.mTimerStartFlag:
            if len(self.mListLoadData) > self.mLoadIndex:
                listPack = []
                listPack = self.mListLoadData[self.mLoadIndex]
                for index, item in enumerate(listPack):
                    listPack[index] = int(item, 10)
                self.mPackAfterUnpackArr.append(copy.deepcopy(listPack))
                self.mDataAfterPro.append(copy.deepcopy(listPack))
                self.mLoadIndex = self.mLoadIndex + 1
            else:
                self.mPackAfterUnpackArr.append(copy.deepcopy(self.mDataAfterPro[self.mLoadDataHead]))
                self.mLoadDataHead += 1
                if self.mLoadDataHead >= self.mLoadIndex:
                    self.mLoadDataHead = 0

    def heartShapeFlash(self):
        self.heartLabel.setVisible(not self.heartLabel.isVisible())

    def event(self, event: QtCore.QEvent) -> bool:
        if event.type() == event.StatusTip:
            if event.tip() == "":
                event = QStatusTipEvent(self.statusStr)
        return super().event(event)

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
                    self.formEcg.show()
                else:
                    QMessageBox.information(None, '消息', '串口未打开', QMessageBox.Ok)
        return False

    def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
        sys.exit(0)

    def slot_dataStore(self):
        self.saveData = SaveData(self.filePath)
        self.saveData.saveDataSignal.connect(self.slot_saveData)
        self.saveData.show()

    def slot_playModel(self):
        if self.ser.isOpen():
            self.serialPortTimer.stop()
            self.procDataTimer.stop()
            try:
                self.ser.close()
            except:
                pass
            self.statusStr = "串口已关闭"
            self.statusBar().showMessage(self.statusStr)
            self.clearData()
        self.mPlayFlag = True
        self.playData = PlayData(self.playDataPath)
        self.playData.playDataSignal.connect(self.slot_playData)
        self.playData.show()

    def slot_about(self):
        QMessageBox.information(None, '关于本软件', "LMH&TZZ", QMessageBox.Ok)

    def slot_quit(self):
        app = QApplication.instance()
        app.quit()

    def slot_resp(self, data):
        self.data_send(data)

    def slot_spo2(self, data):
        self.data_send(data)

    def slot_ecg(self, data):
        self.data_send(data)

    def slot_saveData(self, pathStr):
        self.saveDataPath = pathStr
        self.filePath = pathStr

    def slot_playData(self, pathStr):
        self.playDataPath = pathStr
        if self.mPlayFlag:
            self.loadFile()

    def clearData(self):
        self.mPackAfterUnpackArr = []
        self.mECG1WaveList = []
        self.mSPO2WaveList = []
        self.mRespWaveList = []
        self.ecg_min_val, self.ecg_max_val = float('inf'), float('-inf')
        self.resp_min_val, self.resp_max_val = float('inf'), float('-inf')
        self.spo2_min_val, self.spo2_max_val = float('inf'), float('-inf')
        self.scale_update_counter = 0
        self.ecg_sliding_buffer = []
        self.resp_sliding_buffer = []
        self.spo2_sliding_buffer = []