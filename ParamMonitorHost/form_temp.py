# *_* coding : UTF-8 *_*
# 开发团队 ：乐育科技
# 开发人员 ：zcq
# 开发时间 ：2022/6/20 10:29
# 文件名称 ：form_temp.PY
# 开发工具 ：PyCharm
from PyQt5 import QtWidgets
from PyQt5.QtCore import pyqtSignal
from form_temp_ui import Ui_FormTemp
from PackUnpack import PackUnpack


class FormTemp(QtWidgets.QWidget, Ui_FormTemp):
    # 自定义信号
    tempSignal = pyqtSignal(object)

    def __init__(self):
        super(FormTemp, self).__init__()
        # 打包解包类
        self.mPackUnpack = PackUnpack()
        # 定义数据列表，长度为10
        self.dataList = [0] * 10
        self.setupUi(self)
        self.init()
        # 关联槽函数
        self.okButton.clicked.connect(self.close)
        self.cancelButton.clicked.connect(self.close)
        self.prbTypeComboBox.currentIndexChanged.connect(self.setTempPrbType)

    def init(self):
        # 禁用窗口最大化，禁止调整窗口大小
        self.setFixedSize(self.width(), self.height())

    # 探头类型下拉列表单击信号的槽函数
    def setTempPrbType(self):
        self.dataList[:10] = [0] * 10  # 先置0
        self.dataList[0] = 0x12
        self.dataList[1] = 0x80
        self.dataList[2] = self.prbTypeComboBox.currentIndex()
        self.mPackUnpack.packData(self.dataList)
        self.tempSignal.emit(self.dataList)
