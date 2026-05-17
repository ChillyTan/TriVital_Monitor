from PyQt5.Qt import *
import sys
from ParamMonitor import ParamMonitor

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ParamMonitor()
    window.show()
    sys.exit(app.exec_())