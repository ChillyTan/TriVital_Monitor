# *_* coding : UTF-8 *_*
# 开发团队 ：乐育科技
# 开发人员 ：zcq
# 开发时间 ：2022/6/20 10:28
# 文件名称 ：form_ecg.PY 
# 开发工具 ：PyCharm
from PyQt5 import QtWidgets, QtCore, Qt
from PyQt5.QtCore import pyqtSignal
from form_ecg_ui import Ui_FormECG
from PackUnpack import PackUnpack


class FormEcg(QtWidgets.QWidget, Ui_FormECG):
    # 自定义信号
    ecgSignal = pyqtSignal(object)

    def __init__(self):
        super(FormEcg, self).__init__()
        # 打包解包类
        self.mPackUnpack = PackUnpack()
        # 定义数据列表，长度为10
        self.dataList = [0] * 10
        self.setupUi(self)
        self.init()

    def init(self):
        # 禁用窗口最大化，禁止调整窗口大小
        self.setFixedSize(self.width(), self.height())
        # 关联槽函数
        self.ecg1LeadSetComboBox.currentIndexChanged.connect(self.setECG1Lead)
        self.ecg1GainSetComboBox.currentIndexChanged.connect(self.setECG1Gain)
        self.ecg2LeadSetComboBox.currentIndexChanged.connect(self.setECG2Lead)
        self.ecg2GainSetComboBox.currentIndexChanged.connect(self.setECG2Gain)

    # ECG1导联设置下拉列表单击信号的槽函数
    def setECG1Lead(self):
        self.dataList[:10] = [0] * 10  # 先置0
        self.dataList[0] = 0x14
        self.dataList[1] = 0x81
        self.dataList[2] = self.ecg1LeadSetComboBox.currentIndex() + 1
        self.mPackUnpack.packData(self.dataList)
        self.ecgSignal.emit(self.dataList)

    # ECG1增益设置下拉列表单击信号的槽函数
    def setECG1Gain(self):
        self.dataList[:10] = [0] * 10  # 先置0
        self.dataList[0] = 0x10
        self.dataList[1] = 0x83
        self.dataList[2] = self.ecg1GainSetComboBox.currentIndex()
        self.mPackUnpack.packData(self.dataList)
        self.ecgSignal.emit(self.dataList)

    # ECG2导联设置下拉列表单击信号的槽函数
    def setECG2Lead(self):
        self.dataList[:10] = [0] * 10  # 先置0
        self.dataList[0] = 0x10
        self.dataList[1] = 0x81
        self.dataList[2] = (1 << 4) | (self.ecg2LeadSetComboBox.currentIndex() + 1)
        self.mPackUnpack.packData(self.dataList)
        self.ecgSignal.emit(self.dataList)

    # ECG2增益设置下拉列表单击信号的槽函数
    def setECG2Gain(self):
        self.dataList[:10] = [0] * 10  # 先置0
        self.dataList[0] = 0x10
        self.dataList[1] = 0x83
        self.dataList[2] = (1 << 4) | self.ecg2GainSetComboBox.currentIndex()
        self.mPackUnpack.packData(self.dataList)
        self.ecgSignal.emit(self.dataList)


