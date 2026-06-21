from PyQt5.Qt import *
import sys
from ParamMonitor import ParamMonitor

if __name__ == '__main__':
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    default_font = QFont("SimHei", 10, 87)
    app.setFont(default_font)
    app.setStyleSheet("""
        QWidget {
            font-family: "JetBrains Mono", "SimHei", "Microsoft YaHei";
            font-size: 14px;
            font-weight: 900;
            color: #dbeafe;
            background-color: #05070b;
        }
        QDialog, QMainWindow {
            background-color: #05070b;
        }
        QPushButton {
            min-height: 32px;
            padding: 5px 18px;
            border: 1px solid #0891b2;
            border-radius: 4px;
            background-color: #0e2937;
            color: #cffafe;
            font-weight: 900;
        }
        QPushButton:hover {
            background-color: #164e63;
            border-color: #22d3ee;
            color: #ecfeff;
        }
        QPushButton:pressed {
            background-color: #155e75;
        }
        QComboBox, QLineEdit {
            min-height: 30px;
            padding: 4px 9px;
            border: 1px solid #334155;
            border-radius: 4px;
            background-color: #020617;
            color: #e2e8f0;
            selection-background-color: #164e63;
            selection-color: #ecfeff;
        }
        QComboBox:focus, QLineEdit:focus {
            border-color: #22d3ee;
        }
        QGroupBox {
            font-weight: 900;
            border: 1px solid #334155;
            border-radius: 4px;
            margin-top: 12px;
            padding-top: 10px;
            color: #67e8f9;
            background-color: #0b1120;
        }
        QGroupBox::title {
            subcontrol-origin: margin;
            left: 12px;
            padding: 0 4px;
            color: #67e8f9;
            background-color: #0b1120;
        }
        QMessageBox { background-color: #0b1120; color: #e2e8f0; }
    """)
    window = ParamMonitor()
    window.show()
    sys.exit(app.exec_())
