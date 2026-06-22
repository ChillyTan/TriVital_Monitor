
import sys
import copy
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import QTimer, Qt, QRect, QPoint
from PyQt5.QtGui import QStatusTipEvent, QPixmap, QPainter, QPen, QColor, QIcon
from PyQt5.QtWidgets import QMessageBox, QApplication, QAction
from PyQt5 import QtGui
from ParamMonitor_ui import Ui_MainWindow
from form_setuart import UartSet
import serial
from PackUnpack import PackUnpack

class ParamMonitor(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(ParamMonitor, self).__init__()
        self.setupUi(self)
        self.setup_responsive_ui()
        self.init()
        self.ser = serial.Serial()
        self.mPackUnpck = PackUnpack()
        self.mPackAfterUnpackArr = []
        self.mRespWaveList = []
        self.mRespXStep = 0
        self.mSPO2WaveList = []
        self.mSPO2XStep = 0
        self.mECG1WaveList = []
        self.mECG1XStep = 0
        self._wave_resize_pending = False
        self.create_wave_pixmaps()
        self.convert_signed_16bit = lambda high_byte, low_byte: (high_byte << 8 | low_byte) if (high_byte << 8 | low_byte) < 32768 else (high_byte << 8 | low_byte) - 65536
        self.adaptive_scale_enabled = True
        self.ecg_min_val, self.ecg_max_val = float('inf'), float('-inf')
        self.resp_min_val, self.resp_max_val = float('inf'), float('-inf')
        self.spo2_min_val, self.spo2_max_val = float('inf'), float('-inf')
        self.scale_update_counter = 0
        self.scale_update_interval = 50
        self.ecg_sliding_window_size = 600
        self.resp_sliding_window_size = 2000
        self.spo2_sliding_window_size = 300
        self.ecg_sliding_buffer = []
        self.resp_sliding_buffer = []
        self.spo2_sliding_buffer = []
        self.sync_error_count = 0
        self.sync_error_threshold = 5
        self.heart_icon_visible = True

    def init(self):
        self.menu1 = QAction(self)
        self.menu1.setText('串口设置')
        self.menubar.addAction(self.menu1)
        self.menu1.triggered.connect(self.slot_serialSet)
        self.menu4 = QAction(self)
        self.menu4.setText('关于')
        self.menubar.addAction(self.menu4)
        self.menu4.triggered.connect(self.slot_about)
        self.menu5 = QAction(self)
        self.menu5.setText('退出')
        self.menubar.addAction(self.menu5)
        self.menu5.triggered.connect(self.slot_quit)
        self.statusStr = '等待连接串口'
        self.statusBar().showMessage(self.statusStr)
        self.serialPortTimer = QTimer(self)
        self.serialPortTimer.timeout.connect(self.data_receive)
        self.procDataTimer = QTimer(self)
        self.procDataTimer.timeout.connect(self.data_process)
        self.heartShapeTimer = QTimer(self)
        self.heartShapeTimer.timeout.connect(self.heartShapeFlash)
        self.heartShapeTimer.start(1000)

    def setup_responsive_ui(self):
        self.setWindowTitle("TriVital Monitor")
        self.setMinimumSize(980, 600)
        self.resize(1280, 760)

        zh_font_family = '"SimHei", "Microsoft YaHei"'
        mono_font_family = '"JetBrains Mono"'
        mixed_font_family = '"JetBrains Mono", "SimHei", "Microsoft YaHei"'
        base_font = QtGui.QFont("SimHei", 20, 87)
        title_font = QtGui.QFont("JetBrains Mono", 32, 87)
        value_font = QtGui.QFont("JetBrains Mono", 40, 87)
        self.setFont(base_font)

        self.setStyleSheet(f"""
            QWidget {{
                color: #DCDFE4;
                font-family: {mixed_font_family};
                font-weight: 900;
            }}
            QLabel {{
                background: transparent;
                font-family: {zh_font_family};
                font-weight: 900;
            }}
            QMenuBar {{
                font-family: {zh_font_family};
                font-weight: 900;
                font-size: 14px;
                background: #21252B;
                padding: 3px 10px;
                border-bottom: 1px solid #3E4451;
                color: #DCDFE4;
            }}
            QMenuBar::item {{
                padding: 7px 14px;
                border-radius: 4px;
            }}
            QMenuBar::item:selected {{
                background-color: #3B4048;
                color: #ffffff;
            }}
            QStatusBar {{
                font-family: {mixed_font_family};
                font-weight: 900;
                font-size: 14px;
                background: #21252B;
                border-top: 1px solid #3E4451;
                color: #61AFEF;
            }}
            QGroupBox {{
                background-color: #21252B;
                border: 1px solid #3E4451;
                border-radius: 6px;
                margin-top: 0px;
            }}
            QGroupBox#metricCard {{
                background-color: #21252B;
                border: 1px solid #3E4451;
                border-radius: 6px;
            }}
            QLabel#waveTitle {{
                color: #61AFEF;
                font-size: 24px;
                font-family: {mono_font_family};
                font-weight: 900;
                padding-left: 4px;
            }}
            QLabel#waveLabel {{
                background-color: #1E2127;
                border: 1px solid #3E4451;
                border-radius: 2px;
            }}
            QLabel#metricName {{
                color: #7F848E;
                font-size: 24px;
                font-family: {zh_font_family};
                font-weight: 900;
            }}
            QLabel#metricUnit {{
                color: #ABB2BF;
                font-size: 24px;
                font-family: {mono_font_family};
                font-weight: 900;
            }}
            QLabel#metricValue {{
                font-size: 50px;
                font-family: {mono_font_family};
                font-weight: 900;
            }}
            QLabel#statusText {{
                padding: 5px 8px;
                border: 1px solid #98C379;
                border-radius: 4px;
                background-color: #223024;
                color: #98C379;
                font-size: 20px;
                font-family: {zh_font_family};
                font-weight: 900;
            }}
        """)

        self.centralwidget.setStyleSheet("background-color: #282C34;")
        self.menubar.setStyleSheet(f"""
            QMenuBar {{
                background-color: #21252B;
                color: #DCDFE4;
                border-bottom: 1px solid #3E4451;
                font-family: {zh_font_family};
                font-size: 16px;
                font-weight: 900;
                padding: 4px 10px;
            }}
            QMenuBar::item {{
                padding: 7px 14px;
                border-radius: 4px;
            }}
            QMenuBar::item:selected {{
                background-color: #3B4048;
                color: #ffffff;
            }}
        """)
        self.statusBar().setStyleSheet(f"""
            QStatusBar {{
                background-color: #21252B;
                color: #61AFEF;
                border-top: 1px solid #3E4451;
                font-family: {zh_font_family};
                font-size: 16px;
                font-weight: 900;
            }}
        """)

        wave_title_styles = (
            (self.ecg1Label, "#98C379"),
            (self.spo2Label, "#56B6C2"),
            (self.respLabel, "#E5C07B"),
        )
        for label, title_color in wave_title_styles:
            label.setObjectName("waveTitle")
            label.setFont(title_font)
            label.setMinimumHeight(28)
            label.setStyleSheet(f"""
                color: {title_color};
                background: transparent;
                font-size: 24px;
                font-family: {mono_font_family};
                font-weight: 900;
                padding-left: 6px;
            """)

        for wave_label in (self.ecg1WaveLabel, self.spo2WaveLabel, self.respWaveLabel):
            wave_label.setObjectName("waveLabel")
            wave_label.setText("")
            wave_label.setMinimumHeight(135)
            wave_label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            wave_label.setStyleSheet("""
                background-color: #1E2127;
                border: 1px solid #3E4451;
                border-radius: 2px;
            """)

        for group_box in (self.ecgInfoGroupBox, self.spo2InfoGroupBox, self.respInfoGroupBox):
            group_box.setObjectName("metricCard")
            group_box.setStyleSheet("""
                QGroupBox {
                    background-color: #21252B;
                    border: 1px solid #3E4451;
                    border-radius: 6px;
                    margin-top: 0px;
                }
            """)
            group_box.setMinimumWidth(245)
            group_box.setMaximumWidth(330)
            group_box.setMinimumHeight(160)
            group_box.setSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Expanding)
            shadow = QtWidgets.QGraphicsDropShadowEffect(group_box)
            shadow.setBlurRadius(20)
            shadow.setOffset(0, 0)
            shadow.setColor(QColor(0, 0, 0, 120))
            group_box.setGraphicsEffect(shadow)

        self.heartRateTextLabel.setText("心率")
        self.spo2InfoLabel.setText("血氧")
        self.respUnitLabel.setText("呼吸")
        self.respBpmLabel = QtWidgets.QLabel("bpm", self.respInfoGroupBox)

        metric_labels = (
            self.heartRateTextLabel, self.heartRateLabel, self.heartRateUnitLabel,
            self.spo2InfoLabel, self.labelSPO2Data, self.spo2UnitLabel,
            self.respUnitLabel, self.respRateLabel, self.respBpmLabel,
            self.labelecg_status, self.labelspo2_status, self.labelresp_status,
        )
        for label in metric_labels:
            label.setStyleSheet("")
            label.setAutoFillBackground(False)

        for value_label in (self.heartRateLabel, self.labelSPO2Data, self.respRateLabel):
            value_label.setObjectName("metricValue")
            value_label.setFont(value_font)
            value_label.setAlignment(Qt.AlignCenter)
        self.heartRateLabel.setStyleSheet("color: #98C379; background: transparent;")
        self.labelSPO2Data.setStyleSheet("color: #56B6C2; background: transparent;")
        self.respRateLabel.setStyleSheet("color: #E5C07B; background: transparent;")

        for name_label in (self.heartRateTextLabel, self.spo2InfoLabel, self.respUnitLabel):
            name_label.setObjectName("metricName")
            name_label.setAlignment(Qt.AlignCenter)
            name_label.setStyleSheet(f"""
                color: #7F848E;
                background: transparent;
                font-size: 24px;
                font-family: {zh_font_family};
                font-weight: 900;
            """)

        for unit_label in (self.heartRateUnitLabel, self.spo2UnitLabel, self.respBpmLabel):
            unit_label.setObjectName("metricUnit")
            unit_label.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
            unit_label.setStyleSheet(f"""
                color: #ABB2BF;
                background: transparent;
                font-size: 24px;
                font-family: {mono_font_family};
                font-weight: 900;
            """)

        for status_label in (self.labelecg_status, self.labelspo2_status, self.labelresp_status):
            status_label.setObjectName("statusText")
            status_label.setAlignment(Qt.AlignCenter)
            status_label.setText("等待信号")
            status_label.setStyleSheet(f"""
                color: #98C379;
                background-color: #223024;
                border: 1px solid #98C379;
                border-radius: 4px;
                padding: 5px 8px;
                font-size: 20px;
                font-family: {zh_font_family};
                font-weight: 900;
            """)

        self.heartRateLabel.setText("---")
        self.labelSPO2Data.setText("---")
        self.respRateLabel.setText("---")

        self.heartLabel.setStyleSheet("background: transparent;")
        self.heartLabel.setPixmap(QIcon(":/new/prefix1/image/heart.png").pixmap(28, 28))
        self.heartLabel.setScaledContents(False)
        self.heartLabel.setFixedSize(28, 28)
        self.heartOpacityEffect = QtWidgets.QGraphicsOpacityEffect(self.heartLabel)
        self.heartOpacityEffect.setOpacity(1.0)
        self.heartLabel.setGraphicsEffect(self.heartOpacityEffect)

        self._setup_info_group_layouts()
        main_layout = QtWidgets.QVBoxLayout(self.centralwidget)
        main_layout.setContentsMargins(14, 10, 14, 12)
        main_layout.setSpacing(8)

        header_layout = QtWidgets.QHBoxLayout()
        self.titleWaveLabel = QtWidgets.QLabel("TriVital Monitor")
        self.titleMetricLabel = QtWidgets.QLabel("实时监护")
        self.titleWaveLabel.setStyleSheet(f"color: #61AFEF; font-size: 32px; font-family: {mono_font_family}; font-weight: 900;")
        self.titleMetricLabel.setStyleSheet(f"color: #98C379; font-size: 32px; font-family: {zh_font_family}; font-weight: 900;")
        self.titleWaveLabel.setFont(QtGui.QFont("JetBrains Mono", 22, 87))
        self.titleMetricLabel.setFont(QtGui.QFont("SimHei", 15, 87))
        header_layout.addWidget(self.titleWaveLabel, 1)
        header_layout.addWidget(self.titleMetricLabel, 0, Qt.AlignRight)
        main_layout.addLayout(header_layout)

        body_layout = QtWidgets.QVBoxLayout()
        body_layout.setSpacing(8)

        row_configs = (
            (self.ecg1Label, self.ecg1WaveLabel, self.ecgInfoGroupBox),
            (self.spo2Label, self.spo2WaveLabel, self.spo2InfoGroupBox),
            (self.respLabel, self.respWaveLabel, self.respInfoGroupBox),
        )
        for title_label, wave_label, info_box in row_configs:
            row_layout = QtWidgets.QHBoxLayout()
            row_layout.setSpacing(16)

            wave_section = QtWidgets.QVBoxLayout()
            wave_section.setSpacing(5)
            wave_section.addWidget(title_label)
            wave_section.addWidget(wave_label, 1)

            info_section = QtWidgets.QVBoxLayout()
            info_section.setSpacing(5)
            info_section.addSpacing(title_label.minimumHeight() + wave_section.spacing())
            info_section.addWidget(info_box, 1)

            row_layout.addLayout(wave_section, 1)
            row_layout.addLayout(info_section, 0)
            body_layout.addLayout(row_layout, 1)

        main_layout.addLayout(body_layout, 1)

    def _setup_info_group_layouts(self):
        ecg_layout = QtWidgets.QGridLayout(self.ecgInfoGroupBox)
        ecg_layout.setContentsMargins(16, 12, 16, 12)
        ecg_layout.setHorizontalSpacing(8)
        ecg_layout.setVerticalSpacing(6)
        ecg_layout.addWidget(self.heartRateTextLabel, 0, 0, 1, 3, Qt.AlignCenter)
        ecg_layout.addWidget(self.heartRateLabel, 1, 0, 1, 2, Qt.AlignRight | Qt.AlignVCenter)
        ecg_layout.addWidget(self.heartRateUnitLabel, 1, 2, Qt.AlignLeft | Qt.AlignVCenter)
        ecg_layout.addWidget(self.heartLabel, 2, 0, Qt.AlignRight | Qt.AlignVCenter)
        ecg_layout.addWidget(self.labelecg_status, 2, 1, 1, 2, Qt.AlignLeft | Qt.AlignVCenter)
        ecg_layout.setRowStretch(1, 1)

        spo2_layout = QtWidgets.QGridLayout(self.spo2InfoGroupBox)
        spo2_layout.setContentsMargins(16, 12, 16, 12)
        spo2_layout.setHorizontalSpacing(8)
        spo2_layout.setVerticalSpacing(6)
        spo2_layout.addWidget(self.spo2InfoLabel, 0, 0, 1, 3, Qt.AlignCenter)
        spo2_layout.addWidget(self.labelSPO2Data, 1, 0, 1, 2, Qt.AlignRight | Qt.AlignVCenter)
        spo2_layout.addWidget(self.spo2UnitLabel, 1, 2, Qt.AlignLeft | Qt.AlignVCenter)
        spo2_layout.addWidget(self.labelspo2_status, 2, 0, 1, 3, Qt.AlignCenter)
        spo2_layout.setRowStretch(1, 1)

        resp_layout = QtWidgets.QGridLayout(self.respInfoGroupBox)
        resp_layout.setContentsMargins(16, 12, 16, 12)
        resp_layout.setHorizontalSpacing(8)
        resp_layout.setVerticalSpacing(6)
        resp_layout.addWidget(self.respUnitLabel, 0, 0, 1, 3, Qt.AlignCenter)
        resp_layout.addWidget(self.respRateLabel, 1, 0, 1, 2, Qt.AlignRight | Qt.AlignVCenter)
        resp_layout.addWidget(self.respBpmLabel, 1, 2, Qt.AlignLeft | Qt.AlignVCenter)
        resp_layout.addWidget(self.labelresp_status, 2, 0, 1, 3, Qt.AlignCenter)
        resp_layout.setRowStretch(1, 1)

    def _end_painter(self, painter_name):
        painter = getattr(self, painter_name, None)
        if painter is not None and painter.isActive():
            painter.end()

    def create_wave_pixmaps(self):
        self._end_painter('painterResp')
        self._end_painter('painterSPO2')
        self._end_painter('painterEcg1')

        self.maxRespLength = max(1, self.respWaveLabel.width())
        self.maxRespHeight = max(1, self.respWaveLabel.height())
        self.pixmapResp = QPixmap(self.maxRespLength, self.maxRespHeight)
        self.pixmapResp.fill(QColor("#1E2127"))
        self.painterResp = QPainter(self.pixmapResp)
        self._draw_wave_grid(self.painterResp, self.maxRespLength, self.maxRespHeight)
        self.respWaveLabel.setPixmap(self.pixmapResp)

        self.maxSPO2Length = max(1, self.spo2WaveLabel.width())
        self.maxSPO2Height = max(1, self.spo2WaveLabel.height())
        self.pixmapSPO2 = QPixmap(self.maxSPO2Length, self.maxSPO2Height)
        self.pixmapSPO2.fill(QColor("#1E2127"))
        self.painterSPO2 = QPainter(self.pixmapSPO2)
        self._draw_wave_grid(self.painterSPO2, self.maxSPO2Length, self.maxSPO2Height)
        self.spo2WaveLabel.setPixmap(self.pixmapSPO2)

        self.maxECG1Length = max(1, self.ecg1WaveLabel.width())
        self.maxECG1Height = max(1, self.ecg1WaveLabel.height())
        self.pixmapECG1 = QPixmap(self.maxECG1Length, self.maxECG1Height)
        self.pixmapECG1.fill(QColor("#1E2127"))
        self.painterEcg1 = QPainter(self.pixmapECG1)
        self._draw_wave_grid(self.painterEcg1, self.maxECG1Length, self.maxECG1Height)
        self.ecg1WaveLabel.setPixmap(self.pixmapECG1)

        self.mRespXStep = min(self.mRespXStep, self.maxRespLength - 1)
        self.mSPO2XStep = min(self.mSPO2XStep, self.maxSPO2Length - 1)
        self.mECG1XStep = min(self.mECG1XStep, self.maxECG1Length - 1)

    def _clear_wave_region(self, painter, x, width, height):
        painter.setBrush(QColor("#1E2127"))
        painter.setPen(QPen(QColor("#1E2127"), 1, Qt.SolidLine))
        painter.drawRect(QRect(x, 0, width, height))

    def _draw_wave_grid(self, painter, width, height):
        painter.setPen(QPen(QColor(62, 68, 81, 170), 1, Qt.SolidLine))
        for y in range(0, height, max(22, height // 6)):
            painter.drawLine(0, y, width, y)
        for x in range(0, width, 50):
            painter.drawLine(x, 0, x, height)
        painter.setPen(QPen(QColor(171, 178, 191, 120), 1, Qt.SolidLine))
        for y in range(0, height, max(88, height // 2)):
            painter.drawLine(0, y, width, y)
        for x in range(0, width, 250):
            painter.drawLine(x, 0, x, height)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        if hasattr(self, 'respWaveLabel') and not getattr(self, '_wave_resize_pending', False):
            self._wave_resize_pending = True
            QTimer.singleShot(80, self._handle_wave_resize)

    def _handle_wave_resize(self):
        self._wave_resize_pending = False
        if hasattr(self, 'pixmapResp'):
            self.create_wave_pixmaps()

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
            self.statusStr = "等待连接串口"
            self.statusBar().showMessage(self.statusStr)
        else:
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
            self.statusStr = "连接成功"
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
            if len(self.ecg_sliding_buffer) > self.ecg_sliding_window_size:
                self.ecg_sliding_buffer.pop(0)
            if len(self.resp_sliding_buffer) > self.resp_sliding_window_size:
                self.resp_sliding_buffer.pop(0)
            if len(self.spo2_sliding_buffer) > self.spo2_sliding_window_size:
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
        leadecg = ecg_lead_status

        resp_lead_status = data[4]
        leadresp = resp_lead_status

        spo2_lead_status = data[6]
        leadspo2 = spo2_lead_status

        if leadecg:
            self.labelecg_status.setText("导联正常")
            self.labelecg_status.setStyleSheet("color: #98C379; border: 1px solid #98C379; background-color: #223024; border-radius: 4px; padding: 5px 8px;")
        else:
            self.labelecg_status.setText("导联异常")
            self.labelecg_status.setStyleSheet("color: #E06C75; border: 1px solid #E06C75; background-color: #3A2228; border-radius: 4px; padding: 5px 8px;")

        if leadresp:
            self.labelresp_status.setText("导联正常")
            self.labelresp_status.setStyleSheet("color: #98C379; border: 1px solid #98C379; background-color: #223024; border-radius: 4px; padding: 5px 8px;")
        else:
            self.labelresp_status.setText("导联异常")
            self.labelresp_status.setStyleSheet("color: #E06C75; border: 1px solid #E06C75; background-color: #3A2228; border-radius: 4px; padding: 5px 8px;")

        if leadspo2:
            self.labelspo2_status.setText("导联正常")
            self.labelspo2_status.setStyleSheet("color: #98C379; border: 1px solid #98C379; background-color: #223024; border-radius: 4px; padding: 5px 8px;")
        else:
            self.labelspo2_status.setText("导联异常")
            self.labelspo2_status.setStyleSheet("color: #E06C75; border: 1px solid #E06C75; background-color: #3A2228; border-radius: 4px; padding: 5px 8px;")

    def drawRespWave(self):
        iCnt = len(self.mRespWaveList)
        if iCnt == 0:
            return
        if self.adaptive_scale_enabled and self.scale_update_counter >= self.scale_update_interval:
            if self.resp_max_val == self.resp_min_val:
                self.resp_min_val -= 100
                self.resp_max_val += 100
        self.painterResp.setRenderHint(QPainter.Antialiasing, True)
        if iCnt >= self.maxRespLength - self.mRespXStep:
            self._clear_wave_region(self.painterResp, self.mRespXStep, self.maxRespLength - self.mRespXStep, self.maxRespHeight)
            self._clear_wave_region(self.painterResp, 0, 10 + iCnt - (self.maxRespLength - self.mRespXStep), self.maxRespHeight)
        else:
            self._clear_wave_region(self.painterResp, self.mRespXStep, iCnt + 10, self.maxRespHeight)
        self._draw_wave_grid(self.painterResp, self.maxRespLength, self.maxRespHeight)
        self.painterResp.setPen(QPen(QColor("#E5C07B"), 2, Qt.SolidLine))
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
        self.painterSPO2.setRenderHint(QPainter.Antialiasing, True)
        if iCnt >= self.maxSPO2Length - self.mSPO2XStep:
            self._clear_wave_region(self.painterSPO2, self.mSPO2XStep, self.maxSPO2Length - self.mSPO2XStep, self.maxSPO2Height)
            self._clear_wave_region(self.painterSPO2, 0, 10 + iCnt - (self.maxSPO2Length - self.mSPO2XStep), self.maxSPO2Height)
        else:
            self._clear_wave_region(self.painterSPO2, self.mSPO2XStep, iCnt + 10, self.maxSPO2Height)
        self._draw_wave_grid(self.painterSPO2, self.maxSPO2Length, self.maxSPO2Height)
        self.painterSPO2.setPen(QPen(QColor("#56B6C2"), 2, Qt.SolidLine))
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
        self.painterEcg1.setRenderHint(QPainter.Antialiasing, True)
        if iCnt >= self.maxECG1Length - self.mECG1XStep:
            self._clear_wave_region(self.painterEcg1, self.mECG1XStep, self.maxECG1Length - self.mECG1XStep, self.maxECG1Height)
            self._clear_wave_region(self.painterEcg1, 0, 10 + iCnt - (self.maxECG1Length - self.mECG1XStep), self.maxECG1Height)
        else:
            self._clear_wave_region(self.painterEcg1, self.mECG1XStep, iCnt + 10, self.maxECG1Height)
        self._draw_wave_grid(self.painterEcg1, self.maxECG1Length, self.maxECG1Height)
        self.painterEcg1.setPen(QPen(QColor("#98C379"), 2, Qt.SolidLine))
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

    def heartShapeFlash(self):
        self.heart_icon_visible = not self.heart_icon_visible
        self.heartOpacityEffect.setOpacity(1.0 if self.heart_icon_visible else 0.18)

    def event(self, event: QtCore.QEvent) -> bool:
        if event.type() == event.StatusTip:
            if event.tip() == "":
                event = QStatusTipEvent(self.statusStr)
        return super().event(event)

    def closeEvent(self, a0: QtGui.QCloseEvent) -> None:
        self._end_painter('painterResp')
        self._end_painter('painterSPO2')
        self._end_painter('painterEcg1')
        sys.exit(0)

    def slot_about(self):
        QMessageBox.information(None, '关于', "TriVital Monitor\nLMH&TZZ", QMessageBox.Ok)

    def slot_quit(self):
        app = QApplication.instance()
        app.quit()

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

