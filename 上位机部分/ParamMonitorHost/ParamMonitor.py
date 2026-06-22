
import sys
import copy
import logging
import os
import time
from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import QTimer, Qt, QRect, QPoint
from PyQt5.QtGui import QStatusTipEvent, QPixmap, QPainter, QPen, QColor, QIcon
from PyQt5.QtWidgets import QMessageBox, QApplication, QAction
from PyQt5 import QtGui
from ParamMonitor_ui import Ui_MainWindow
from form_setuart import UartSet
import serial
import qtawesome as qta
import qtawesome.iconic_font as qta_iconic
from monitor_alarm import AlarmLimits, evaluate_alarm_state
from PackUnpack import PackUnpack
from ui_theme import (
    COLORS,
    MONO_FONT,
    alarm_title_style,
    debug_controls_style,
    bold_font,
    debug_dock_style,
    debug_text_style,
    main_window_style,
    menu_bar_style,
    menu_style,
    metric_card_style,
    metric_name_style,
    metric_unit_style,
    metric_value_style,
    status_bar_style,
    status_label_style,
    toolbar_style,
    wave_label_style,
    wave_title_style,
)

qta_iconic.IconicFont._install_fonts = lambda self, fonts_directory, system_wide=False: fonts_directory

class ParamMonitor(QtWidgets.QMainWindow, Ui_MainWindow):
    def __init__(self):
        super(ParamMonitor, self).__init__()
        self.setupUi(self)
        self.setDockOptions(
            QtWidgets.QMainWindow.AllowNestedDocks |
            QtWidgets.QMainWindow.AllowTabbedDocks |
            QtWidgets.QMainWindow.AnimatedDocks
        )
        self.setup_responsive_ui()
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
        self.wave_paused = False
        self.debug_visible = True
        self.debug_output_paused = False
        self.debug_error_only = False
        self.alarm_muted = False
        self.active_alarms = []
        self.last_hr = None
        self.last_resp_rate = None
        self.last_spo2 = None
        self.lead_status = {"ECG": None, "RESP": None, "SpO2": None}
        self.rx_bytes = 0
        self.rx_packets = 0
        self.checksum_error_count = 0
        self.packet_counts = {0x10: 0, 0x11: 0, 0x12: 0}
        self.start_time = time.time()
        self.last_packet_time = None
        self.current_port_label = "未连接"
        self.current_baudrate = ""
        self.alarm_limits = AlarmLimits()
        self.setup_logger()
        self.init()

    def init(self):
        self.menubar.setVisible(False)
        self.menu1 = QAction(self)
        self.menu1.setText('串口设置')
        self.menu1.triggered.connect(self.slot_serialSet)
        self.menu4 = QAction(self)
        self.menu4.setText('关于')
        self.menu4.triggered.connect(self.slot_about)
        self.menu5 = QAction(self)
        self.menu5.setText('退出')
        self.menu5.triggered.connect(self.slot_quit)
        self.statusStr = '等待连接串口'
        self.statusBar().showMessage(self.statusStr)
        self.setup_toolbar()
        self.setup_debug_dock()
        self.serialPortTimer = QTimer(self)
        self.serialPortTimer.timeout.connect(self.data_receive)
        self.procDataTimer = QTimer(self)
        self.procDataTimer.timeout.connect(self.data_process)
        self.statusTimer = QTimer(self)
        self.statusTimer.timeout.connect(self.update_status_bar)
        self.statusTimer.start(500)
        self.heartShapeTimer = QTimer(self)
        self.heartShapeTimer.timeout.connect(self.heartShapeFlash)
        self.heartShapeTimer.start(1000)
        self.update_status_bar()

    def setup_logger(self):
        log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
        os.makedirs(log_dir, exist_ok=True)
        log_path = os.path.join(log_dir, "host_monitor.log")
        self.logger = logging.getLogger("TriVitalMonitor")
        self.logger.setLevel(logging.INFO)
        if not self.logger.handlers:
            handler = logging.FileHandler(log_path, encoding="utf-8")
            handler.setFormatter(logging.Formatter("%(asctime)s [%(levelname)s] %(message)s"))
            self.logger.addHandler(handler)

    def icon(self, name, color="#DCDFE4"):
        return qta.icon(name, color=color, color_active="#FFFFFF")

    def setup_toolbar(self):
        self.toolbar = QtWidgets.QToolBar("监护工具", self)
        self.toolbar.setMovable(False)
        self.toolbar.setIconSize(QtCore.QSize(18, 18))
        self.toolbar.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.toolbar.setStyleSheet(toolbar_style())
        self.addToolBar(Qt.TopToolBarArea, self.toolbar)

        self.actionSerialToolbar = QAction(self.icon("fa5s.plug", "#61AFEF"), "串口", self)
        self.actionSerialToolbar.triggered.connect(self.slot_serialSet)
        self.toolbar.addAction(self.actionSerialToolbar)

        self.actionPauseWave = QAction(self.icon("fa5s.pause", "#E5C07B"), "暂停波形", self)
        self.actionPauseWave.setCheckable(True)
        self.actionPauseWave.triggered.connect(self.toggle_wave_pause)
        self.toolbar.addAction(self.actionPauseWave)

        self.actionClearWave = QAction(self.icon("fa5s.eraser", "#ABB2BF"), "清屏", self)
        self.actionClearWave.triggered.connect(self.clear_wave_screen)
        self.toolbar.addAction(self.actionClearWave)

        self.actionMuteAlarm = QAction(self.icon("fa5s.bell-slash", "#E06C75"), "报警静音", self)
        self.actionMuteAlarm.setCheckable(True)
        self.actionMuteAlarm.triggered.connect(self.toggle_alarm_mute)
        self.toolbar.addAction(self.actionMuteAlarm)

        self.viewMenu = QtWidgets.QMenu("视图", self)
        self.viewMenu.setStyleSheet(menu_style())

        self.actionDebugPanel = QAction(self.icon("fa5s.terminal", "#56B6C2"), "显示协议调试", self)
        self.actionDebugPanel.setCheckable(True)
        self.actionDebugPanel.setChecked(True)
        self.actionDebugPanel.triggered.connect(self.toggle_debug_panel)
        self.viewMenu.addAction(self.actionDebugPanel)

        self.actionDockDebugLeft = QAction(self.icon("fa5s.arrow-left", "#ABB2BF"), "调试左侧", self)
        self.actionDockDebugLeft.triggered.connect(lambda: self.dock_debug_panel(Qt.LeftDockWidgetArea))
        self.viewMenu.addAction(self.actionDockDebugLeft)

        self.actionDockDebugRight = QAction(self.icon("fa5s.arrow-right", "#ABB2BF"), "调试右侧", self)
        self.actionDockDebugRight.triggered.connect(lambda: self.dock_debug_panel(Qt.RightDockWidgetArea))
        self.viewMenu.addAction(self.actionDockDebugRight)

        self.viewToolButton = QtWidgets.QToolButton(self)
        self.viewToolButton.setIcon(self.icon("fa5s.layer-group", "#61AFEF"))
        self.viewToolButton.setText("视图")
        self.viewToolButton.setToolButtonStyle(Qt.ToolButtonTextBesideIcon)
        self.viewToolButton.setPopupMode(QtWidgets.QToolButton.InstantPopup)
        self.viewToolButton.setMenu(self.viewMenu)
        self.toolbar.addWidget(self.viewToolButton)

        self.toolbar.addSeparator()

        self.actionAboutToolbar = QAction(self.icon("fa5s.info-circle", "#ABB2BF"), "关于", self)
        self.actionAboutToolbar.triggered.connect(self.slot_about)
        self.toolbar.addAction(self.actionAboutToolbar)

        self.actionQuitToolbar = QAction(self.icon("fa5s.sign-out-alt", "#E06C75"), "退出", self)
        self.actionQuitToolbar.triggered.connect(self.slot_quit)
        self.toolbar.addAction(self.actionQuitToolbar)

    def setup_debug_dock(self):
        self.debugDock = QtWidgets.QDockWidget("协议调试", self)
        self.debugDock.setAllowedAreas(Qt.LeftDockWidgetArea | Qt.RightDockWidgetArea)
        self.debugDock.setFeatures(
            QtWidgets.QDockWidget.DockWidgetClosable |
            QtWidgets.QDockWidget.DockWidgetMovable |
            QtWidgets.QDockWidget.DockWidgetFloatable
        )
        debug_widget = QtWidgets.QWidget()
        debug_widget.setStyleSheet(f"background-color: {COLORS['panel']}; color: {COLORS['text']};")
        debug_layout = QtWidgets.QVBoxLayout(debug_widget)
        debug_layout.setContentsMargins(8, 8, 8, 8)
        debug_layout.setSpacing(6)

        self.protocolStatsLabel = QtWidgets.QLabel()
        self.protocolStatsLabel.setStyleSheet(f"color: {COLORS['text_muted']}; font-family: {MONO_FONT}; font-size: 12px;")
        self.debugTextEdit = QtWidgets.QPlainTextEdit()
        self.debugTextEdit.setReadOnly(True)
        self.debugTextEdit.setMaximumBlockCount(300)
        self.debugTextEdit.setStyleSheet(debug_text_style())
        debug_layout.addWidget(self.protocolStatsLabel)
        debug_layout.addWidget(self.debugTextEdit, 1)

        controls_group = QtWidgets.QGroupBox("DEBUG OUTPUT")
        controls_group.setStyleSheet(debug_controls_style())
        controls_layout = QtWidgets.QGridLayout(controls_group)
        controls_layout.setContentsMargins(15, 28, 15, 14)
        controls_layout.setVerticalSpacing(10)
        controls_layout.setHorizontalSpacing(10)

        self.pauseDebugButton = QtWidgets.QPushButton("暂停输出")
        self.pauseDebugButton.setCheckable(True)
        self.pauseDebugButton.setCursor(Qt.PointingHandCursor)
        self.pauseDebugButton.toggled.connect(self.toggle_debug_output_pause)

        self.errorOnlyButton = QtWidgets.QPushButton("只看错误")
        self.errorOnlyButton.setObjectName("errorOnlyButton")
        self.errorOnlyButton.setCheckable(True)
        self.errorOnlyButton.setCursor(Qt.PointingHandCursor)
        self.errorOnlyButton.toggled.connect(self.toggle_debug_error_only)

        self.clearDebugButton = QtWidgets.QPushButton("清空调试")
        self.clearDebugButton.setObjectName("clearDebugButton")
        self.clearDebugButton.setCursor(Qt.PointingHandCursor)
        self.clearDebugButton.clicked.connect(self.clear_debug_output)

        controls_layout.addWidget(self.pauseDebugButton, 0, 0)
        controls_layout.addWidget(self.errorOnlyButton, 0, 1)
        controls_layout.addWidget(self.clearDebugButton, 1, 0, 1, 2)
        debug_layout.addWidget(controls_group)

        self.debugDock.setWidget(debug_widget)
        self.debugDock.setStyleSheet(debug_dock_style())
        self.debugDock.visibilityChanged.connect(self.actionDebugPanel.setChecked)
        self.addDockWidget(Qt.RightDockWidgetArea, self.debugDock)
        self.resizeDocks([self.debugDock], [360], Qt.Horizontal)

    def append_debug_log(self, text, level="info"):
        if not hasattr(self, "debugTextEdit"):
            return
        if self.debug_output_paused:
            return
        if self.debug_error_only and level != "error":
            return
        timestamp = time.strftime("%H:%M:%S")
        self.debugTextEdit.appendPlainText(f"{timestamp} {text}")

    def toggle_debug_output_pause(self, checked):
        self.debug_output_paused = checked
        self.pauseDebugButton.setText("继续输出" if checked else "暂停输出")

    def toggle_debug_error_only(self, checked):
        self.debug_error_only = checked
        self.errorOnlyButton.setText("显示全部" if checked else "只看错误")

    def clear_debug_output(self):
        if hasattr(self, "debugTextEdit"):
            self.debugTextEdit.clear()

    def update_protocol_stats(self):
        if not hasattr(self, "protocolStatsLabel"):
            return
        self.protocolStatsLabel.setText(
            f"RX {self.rx_bytes} B | PACK {self.rx_packets} | "
            f"WAVE {self.packet_counts[0x10]} PARAM {self.packet_counts[0x11]} "
            f"STATUS {self.packet_counts[0x12]} | ERR {self.checksum_error_count}"
        )

    def update_status_bar(self):
        elapsed = int(time.time() - self.start_time)
        alarm_text = "正常" if not self.active_alarms else "报警: " + " / ".join(self.active_alarms[:3])
        paused_text = "暂停" if self.wave_paused else "运行"
        mute_text = "静音" if self.alarm_muted else "响铃"
        self.statusStr = (
            f"串口 {self.current_port_label} {self.current_baudrate} | "
            f"RX {self.rx_bytes}B 包 {self.rx_packets} 错 {self.checksum_error_count} | "
            f"波形 {paused_text} | 报警 {mute_text} {alarm_text} | 运行 {elapsed}s"
        )
        self.statusBar().showMessage(self.statusStr)
        self.update_protocol_stats()

    def setup_responsive_ui(self):
        self.setWindowTitle("TRIVITAL MONITOR")
        self.setMinimumSize(980, 600)
        self.resize(1280, 760)

        base_font = bold_font("SimHei", 20)
        title_font = bold_font("DejaVu Sans Mono", 32)
        value_font = bold_font("DejaVu Sans Mono", 40)
        self.setFont(base_font)

        self.setStyleSheet(main_window_style())
        self.centralwidget.setStyleSheet(f"background-color: {COLORS['window']};")
        self.menubar.setStyleSheet(menu_bar_style())
        self.statusBar().setStyleSheet(status_bar_style())

        wave_title_styles = (
            (self.ecg1Label, COLORS["ecg"]),
            (self.spo2Label, COLORS["spo2"]),
            (self.respLabel, COLORS["resp"]),
        )
        for label, title_color in wave_title_styles:
            label.setObjectName("waveTitle")
            label.setFont(title_font)
            label.setMinimumHeight(28)
            label.setStyleSheet(wave_title_style(title_color))

        for wave_label in (self.ecg1WaveLabel, self.spo2WaveLabel, self.respWaveLabel):
            wave_label.setObjectName("waveLabel")
            wave_label.setText("")
            wave_label.setMinimumHeight(135)
            wave_label.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
            wave_label.setStyleSheet(wave_label_style())

        for group_box in (self.ecgInfoGroupBox, self.spo2InfoGroupBox, self.respInfoGroupBox):
            group_box.setObjectName("metricCard")
            group_box.setStyleSheet(metric_card_style())
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
        self.heartRateLabel.setStyleSheet(metric_value_style(COLORS["ecg"]))
        self.labelSPO2Data.setStyleSheet(metric_value_style(COLORS["spo2"]))
        self.respRateLabel.setStyleSheet(metric_value_style(COLORS["resp"]))

        for name_label in (self.heartRateTextLabel, self.spo2InfoLabel, self.respUnitLabel):
            name_label.setObjectName("metricName")
            name_label.setAlignment(Qt.AlignCenter)
            name_label.setStyleSheet(metric_name_style())
        self.heartRateTextLabel.setStyleSheet(metric_name_style(COLORS["ecg"]))
        self.spo2InfoLabel.setStyleSheet(metric_name_style(COLORS["spo2"]))
        self.respUnitLabel.setStyleSheet(metric_name_style(COLORS["resp"]))

        for unit_label in (self.heartRateUnitLabel, self.spo2UnitLabel, self.respBpmLabel):
            unit_label.setObjectName("metricUnit")
            unit_label.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)
            unit_label.setStyleSheet(metric_unit_style())

        for status_label in (self.labelecg_status, self.labelspo2_status, self.labelresp_status):
            status_label.setObjectName("statusText")
            status_label.setAlignment(Qt.AlignCenter)
            status_label.setText("等待信号")
            status_label.setStyleSheet(status_label_style(False))

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
        self.titleWaveLabel = QtWidgets.QLabel("TRIVITAL MONITOR")
        self.titleMetricLabel = QtWidgets.QLabel("报警状态: 正常")
        self.titleWaveLabel.setStyleSheet(f"color: {COLORS['primary']}; font-size: 32px; font-family: {MONO_FONT}; font-weight: 900;")
        self.titleMetricLabel.setStyleSheet(alarm_title_style(False))
        self.titleWaveLabel.setFont(bold_font("DejaVu Sans Mono", 22))
        self.titleMetricLabel.setFont(bold_font("SimHei", 13))
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
        self.pixmapResp.fill(QColor(COLORS["surface"]))
        self.painterResp = QPainter(self.pixmapResp)
        self._draw_wave_grid(self.painterResp, self.maxRespLength, self.maxRespHeight)
        self.respWaveLabel.setPixmap(self.pixmapResp)

        self.maxSPO2Length = max(1, self.spo2WaveLabel.width())
        self.maxSPO2Height = max(1, self.spo2WaveLabel.height())
        self.pixmapSPO2 = QPixmap(self.maxSPO2Length, self.maxSPO2Height)
        self.pixmapSPO2.fill(QColor(COLORS["surface"]))
        self.painterSPO2 = QPainter(self.pixmapSPO2)
        self._draw_wave_grid(self.painterSPO2, self.maxSPO2Length, self.maxSPO2Height)
        self.spo2WaveLabel.setPixmap(self.pixmapSPO2)

        self.maxECG1Length = max(1, self.ecg1WaveLabel.width())
        self.maxECG1Height = max(1, self.ecg1WaveLabel.height())
        self.pixmapECG1 = QPixmap(self.maxECG1Length, self.maxECG1Height)
        self.pixmapECG1.fill(QColor(COLORS["surface"]))
        self.painterEcg1 = QPainter(self.pixmapECG1)
        self._draw_wave_grid(self.painterEcg1, self.maxECG1Length, self.maxECG1Height)
        self.ecg1WaveLabel.setPixmap(self.pixmapECG1)

        self.mRespXStep = min(self.mRespXStep, self.maxRespLength - 1)
        self.mSPO2XStep = min(self.mSPO2XStep, self.maxSPO2Length - 1)
        self.mECG1XStep = min(self.mECG1XStep, self.maxECG1Length - 1)

    def _clear_wave_region(self, painter, x, width, height):
        painter.setBrush(QColor(COLORS["surface"]))
        painter.setPen(QPen(QColor(COLORS["surface"]), 1, Qt.SolidLine))
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

    def toggle_wave_pause(self, checked):
        self.wave_paused = checked
        self.actionPauseWave.setText("继续波形" if checked else "暂停波形")
        self.append_debug_log("WAVE PAUSE" if checked else "WAVE RESUME")
        self.update_status_bar()

    def toggle_alarm_mute(self, checked):
        self.alarm_muted = checked
        self.actionMuteAlarm.setText("取消静音" if checked else "报警静音")
        self.append_debug_log("ALARM MUTE ON" if checked else "ALARM MUTE OFF")
        self.update_status_bar()

    def toggle_debug_panel(self, checked):
        self.debug_visible = checked
        self.debugDock.setVisible(checked)

    def dock_debug_panel(self, area):
        self.debugDock.setFloating(False)
        self.addDockWidget(area, self.debugDock)
        self.debugDock.show()
        self.debugDock.raise_()
        self.actionDebugPanel.setChecked(True)

    def clear_wave_screen(self):
        self.clearData()
        self.mRespXStep = 0
        self.mSPO2XStep = 0
        self.mECG1XStep = 0
        self.create_wave_pixmaps()
        self.append_debug_log("WAVE CLEAR")
        self.update_status_bar()

    def slot_serialSet(self):
        if self.ser.isOpen():
            self.uartset = UartSet(True)
        else:
            self.uartset = UartSet(False)
        self.uartset.serialSignal.connect(self.slot_serial)
        self.uartset.show()

    def slot_serial(self, portNum, baudRate, dataBits, stopBits, parity):
        if self.ser.isOpen():
            self.disconnect_serial("手动断开串口")
        else:
            self.ser.port = portNum
            self.ser.baudrate = int(baudRate)
            self.ser.bytesize = int(dataBits)
            self.ser.stopbits = int(stopBits)
            self.ser.parity = parity
            try:
                self.ser.open()
            except Exception as exc:
                self.logger.exception("串口打开失败: %s", portNum)
                QMessageBox.critical(self, "串口错误", f"串口打开失败: {exc}")
                return
            self.current_port_label = portNum
            self.current_baudrate = str(baudRate)
            self.statusStr = "连接成功"
            self.logger.info("串口连接成功: %s %s", portNum, baudRate)
            self.append_debug_log(f"OPEN {portNum} {baudRate},{dataBits},{parity},{stopBits}")
            self.serialPortTimer.start(2)
            self.procDataTimer.start(10)
            self.update_status_bar()

    def disconnect_serial(self, reason):
        self.serialPortTimer.stop()
        self.procDataTimer.stop()
        try:
            if self.ser.isOpen():
                self.ser.close()
        except Exception as exc:
            self.logger.warning("关闭串口异常: %s", exc)
        self.current_port_label = "未连接"
        self.current_baudrate = ""
        self.statusStr = "等待连接串口"
        self.append_debug_log(f"CLOSE {reason}")
        self.logger.info("串口断开: %s", reason)
        self.update_status_bar()

    def data_send(self, data):
        if self.ser.isOpen():
            data = bytes(data)
            self.ser.write(data)
        else:
            self.append_debug_log("TX ignored: serial closed", level="error")

    def data_receive(self):
        try:
            num = self.ser.inWaiting()
        except Exception as exc:
            self.logger.warning("串口读取失败: %s", exc)
            self.append_debug_log(f"RX error: {exc}", level="error")
            self.disconnect_serial("串口读取失败")
            QMessageBox.warning(self, "串口断开", f"串口读取失败，已断开连接: {exc}")
            return None
        if num > 0:
            data = self.ser.read(num)
            self.rx_bytes += len(data)
            self.append_debug_log("RX " + " ".join(f"{byte:02X}" for byte in data[:32]) + (" ..." if len(data) > 32 else ""))
            for i in range(0, len(data)):
                byte = data[i]
                if self.mPackUnpck.sGotPackId:
                    if byte < 0x80 and self.mPackUnpck.sPackLen < 10:
                        self.sync_error_count += 1
                        self.checksum_error_count += 1
                        self.append_debug_log(f"SYNC error byte={byte:02X}", level="error")
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
                    self.rx_packets += 1
                    self.last_packet_time = time.time()
                    self.append_debug_log("PACK " + " ".join(f"{value:02X}" for value in temp[:8]))
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
                if module_id in self.packet_counts:
                    self.packet_counts[module_id] += 1

                if module_id == 0x10:
                    self.analyzeWaveData(packet)
                elif module_id == 0x11:
                    self.analyzeParamData(packet)
                elif module_id == 0x12:
                    self.analyzeStatusData(packet)
            del self.mPackAfterUnpackArr[0:num]
        if self.wave_paused:
            return
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
            self.last_hr = hr
            self.heartRateLabel.setText(str(hr))
        else:
            self.last_hr = None
            self.heartRateLabel.setText("---")
        if 0 < resp_rate < 120:
            self.last_resp_rate = resp_rate
            self.respRateLabel.setText(str(resp_rate))
        else:
            self.last_resp_rate = None
            self.respRateLabel.setText("---")
        if 0 <= spo2_value <= 100:
            self.last_spo2 = spo2_value
            self.labelSPO2Data.setText(str(spo2_value))
        else:
            self.last_spo2 = None
            self.labelSPO2Data.setText("---")
        self.evaluate_alarms()

    def analyzeStatusData(self, data):
        ecg_lead_status = data[2]
        leadecg = ecg_lead_status

        resp_lead_status = data[4]
        leadresp = resp_lead_status

        spo2_lead_status = data[6]
        leadspo2 = spo2_lead_status
        self.lead_status["ECG"] = bool(leadecg)
        self.lead_status["RESP"] = bool(leadresp)
        self.lead_status["SpO2"] = bool(leadspo2)

        if leadecg:
            self.labelecg_status.setText("导联正常")
            self.labelecg_status.setStyleSheet(status_label_style(False))
        else:
            self.labelecg_status.setText("导联异常")
            self.labelecg_status.setStyleSheet(status_label_style(True))

        if leadresp:
            self.labelresp_status.setText("导联正常")
            self.labelresp_status.setStyleSheet(status_label_style(False))
        else:
            self.labelresp_status.setText("导联异常")
            self.labelresp_status.setStyleSheet(status_label_style(True))

        if leadspo2:
            self.labelspo2_status.setText("导联正常")
            self.labelspo2_status.setStyleSheet(status_label_style(False))
        else:
            self.labelspo2_status.setText("导联异常")
            self.labelspo2_status.setStyleSheet(status_label_style(True))
        self.evaluate_alarms()

    def set_metric_state(self, label, normal_color, alarm=False, invalid=False):
        if invalid:
            color = COLORS["text_dim"]
        elif alarm:
            color = COLORS["alarm"]
        else:
            color = normal_color
        label.setStyleSheet(metric_value_style(color))

    def evaluate_alarms(self):
        result = evaluate_alarm_state(
            self.last_hr,
            self.last_resp_rate,
            self.last_spo2,
            self.lead_status,
            self.alarm_limits,
        )
        alarms = result.alarms

        previous = set(self.active_alarms)
        current = set(alarms)
        for alarm in sorted(current - previous):
            self.logger.warning("报警触发: %s", alarm)
            self.append_debug_log(f"ALARM {alarm}", level="error")

        self.active_alarms = alarms
        self.set_metric_state(self.heartRateLabel, COLORS["ecg"], result.hr_alarm, self.last_hr is None)
        self.set_metric_state(self.respRateLabel, COLORS["resp"], result.resp_alarm, self.last_resp_rate is None)
        self.set_metric_state(self.labelSPO2Data, COLORS["spo2"], result.spo2_alarm, self.last_spo2 is None)
        self.titleMetricLabel.setText("报警状态: 异常" if alarms else "报警状态: 正常")
        self.titleMetricLabel.setStyleSheet(alarm_title_style(bool(alarms)))
        if alarms and not self.alarm_muted:
            QApplication.beep()
        self.update_status_bar()

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
        self.painterResp.setPen(QPen(QColor(COLORS["resp"]), 2, Qt.SolidLine))
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
        self.painterSPO2.setPen(QPen(QColor(COLORS["spo2"]), 2, Qt.SolidLine))
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
        self.painterEcg1.setPen(QPen(QColor(COLORS["ecg"]), 2, Qt.SolidLine))
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
        QMessageBox.information(None, '关于', "TriVital Monitor\nLMH & TZZ", QMessageBox.Ok)

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

