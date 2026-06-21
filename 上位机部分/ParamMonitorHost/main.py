from PyQt5.Qt import *
import sys
from ParamMonitor import ParamMonitor

if __name__ == '__main__':
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    default_font = QFont("SimHei", 10, 87)
    app.setFont(default_font)
    window = ParamMonitor()
    window.show()
    sys.exit(app.exec_())
