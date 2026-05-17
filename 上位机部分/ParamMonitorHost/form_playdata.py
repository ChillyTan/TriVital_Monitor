# *_* coding : UTF-8 *_*
# 开发团队 ：乐育科技
# 开发人员 ：zcq
# 开发时间 ：2022/6/14 18:16
# 文件名称 ：form_savedata.PY
# 开发工具 ：PyCharm
import os
from PyQt5 import QtWidgets
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtWidgets import QFileDialog
from form_playdata_ui import Ui_FormPlayData


class PlayData(QtWidgets.QWidget, Ui_FormPlayData):
    # 自定义信号
    playDataSignal = pyqtSignal(str)

    def __init__(self, pathStr):
        super(PlayData, self).__init__()
        self.setupUi(self)
        self.init()
        self.dataPath = pathStr
        self.readPathLineEdit.setText(pathStr)
        # 关联槽函数
        self.openButton.clicked.connect(self.getDataPath)
        self.okButton.clicked.connect(self.setPlayDataPath)
        self.cancelButton.clicked.connect(self.close)

    def init(self):
        # 禁用窗口最大化，禁止调整窗口大小
        self.setFixedSize(self.width(), self.height())

    # “打开”按钮单击信号的槽函数
    def getDataPath(self):
        # 获取演示数据路径，存到dataPath
        filename, _ = QFileDialog.getOpenFileName(self, '选择文件', os.getcwd(), "All File(*);;Text Files(*.txt)")
        if len(filename) != 0:
            self.readPathLineEdit.setText(filename)
            self.dataPath = filename

    # “确定”按钮单击信号的槽函数
    def setPlayDataPath(self):
        # 将获取的dataPath传到playDataSignal信号关联的槽函数
        self.playDataSignal.emit(self.dataPath)
        self.close()