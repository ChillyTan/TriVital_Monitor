# *_* coding : UTF-8 *_*
# 开发团队 ：乐育科技
# 开发人员 ：zcq
# 开发时间 ：2022/6/20 10:29
# 文件名称 ：form_spo2.PY 
# 开发工具 ：PyCharm
from PyQt5 import QtWidgets
from PyQt5.QtCore import pyqtSignal
from form_spo2_ui import Ui_FormSPO2
from PackUnpack import PackUnpack

class FormSpo2(QtWidgets.QWidget, Ui_FormSPO2):
    # 自定义信号
    spo2Signal = pyqtSignal(object)

    def __init__(self):
        super(FormSpo2, self).__init__()
        # 打包解包类
        self.mPackUnpack = PackUnpack()
        # 定义数据列表，长度为10
        self.dataList = [0] * 10
        self.setupUi(self)
        self.init()
        # 关联槽函数
        self.okButton.clicked.connect(self.close)
        self.cancelButton.clicked.connect(self.close)
        self.sensComboBox.currentIndexChanged.connect(self.setSPO2Sens)

    def init(self):
        # 禁用窗口最大化，禁止调整窗口大小
        self.setFixedSize(self.width(), self.height())

    # 计算灵敏度下拉列表单击信号的槽函数
    def setSPO2Sens(self):
        self.dataList[:10] = [0] * 10  # 先置0
        self.dataList[0] = 0x13
        self.dataList[1] = 0x80
        self.dataList[2] = self.sensComboBox.currentIndex() + 1
        self.mPackUnpack.packData(self.dataList)
        self.spo2Signal.emit(self.dataList)
