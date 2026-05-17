# *_* coding : UTF-8 *_*
# 开发团队 ：乐育科技
# 开发人员 ：zcq
# 开发时间 ：2022/6/17 11:17
# 文件名称 ：form_nibp.PY 
# 开发工具 ：PyCharm
from PyQt5 import QtWidgets
from PyQt5.QtCore import pyqtSignal
from form_nibp_ui import Ui_FormNIBP
from PackUnpack import PackUnpack


class FormNibp(QtWidgets.QWidget, Ui_FormNIBP):
    # 自定义信号
    nibpSignal = pyqtSignal(object)

    def __init__(self):
        super(FormNibp, self).__init__()
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
        self.startMeasButton.clicked.connect(self.startMeasure)
        self.stopMeasButton.clicked.connect(self.stopMeasure)

    # “开始测量”按钮单击信号的槽函数
    def startMeasure(self):
        self.dataList[0] = 0x14
        self.dataList[1] = 0x80
        self.mPackUnpack.packData(self.dataList)
        self.nibpSignal.emit(self.dataList)
        self.close()

    # “停止测量”按钮单击信号的槽函数
    def stopMeasure(self):
        self.dataList[0] = 0x14
        self.dataList[1] = 0x81
        self.mPackUnpack.packData(self.dataList)
        self.nibpSignal.emit(self.dataList)
        self.close()

