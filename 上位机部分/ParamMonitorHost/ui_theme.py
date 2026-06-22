from PyQt5 import QtGui


ZH_FONT = '"SimHei", "Microsoft YaHei"'
MONO_FONT = '"DejaVu Sans Mono"'
MIXED_FONT = '"DejaVu Sans Mono", "SimHei", "Microsoft YaHei"'

FONT_WEIGHT_BOLD = 87

COLORS = {
    "window": "#282C34",
    "panel": "#21252B",
    "panel_alt": "#23272F",
    "surface": "#1E2127",
    "button": "#2C313A",
    "button_hover": "#333945",
    "button_checked": "#3A4B5F",
    "border": "#3E4451",
    "border_alt": "#414855",
    "text": "#DCDFE4",
    "text_muted": "#ABB2BF",
    "text_dim": "#7F848E",
    "white": "#FFFFFF",
    "primary": "#61AFEF",
    "ecg": "#98C379",
    "spo2": "#56B6C2",
    "resp": "#E5C07B",
    "alarm": "#E06C75",
    "normal_bg": "#223024",
    "alarm_bg": "#3A2228",
}


def bold_font(family, size):
    return QtGui.QFont(family, size, FONT_WEIGHT_BOLD)


def toolbar_style():
    return f"""
        QToolBar {{
            background: {COLORS["panel"]};
            border-bottom: 1px solid {COLORS["border"]};
            spacing: 8px;
            padding: 6px 12px;
        }}
        QToolButton {{
            color: {COLORS["text"]};
            background: {COLORS["button"]};
            border: 1px solid {COLORS["border"]};
            border-radius: 6px;
            padding: 6px 12px;
            font-family: {ZH_FONT};
            font-size: 18px;
            font-weight: 900;
        }}
        QToolButton:hover {{
            border-color: {COLORS["primary"]};
            background: {COLORS["button_hover"]};
        }}
        QToolButton:checked {{
            color: {COLORS["white"]};
            background: {COLORS["button_checked"]};
            border-color: {COLORS["primary"]};
        }}
    """


def menu_style():
    return f"""
        QMenu {{
            background-color: {COLORS["panel"]};
            color: {COLORS["text"]};
            border: 1px solid {COLORS["border"]};
            padding: 4px;
            font-family: {ZH_FONT};
            font-size: 14px;
            font-weight: 900;
        }}
        QMenu::item {{
            padding: 7px 28px 7px 12px;
            background: transparent;
        }}
        QMenu::item:selected {{
            background-color: {COLORS["button_checked"]};
            color: {COLORS["white"]};
        }}
        QMenu::indicator {{
            width: 14px;
            height: 14px;
        }}
    """


def main_window_style():
    return f"""
        QWidget {{
            color: {COLORS["text"]};
            font-family: {MIXED_FONT};
            font-weight: 900;
        }}
        QLabel {{
            background: transparent;
            font-family: {ZH_FONT};
            font-weight: 900;
        }}
        QMenuBar {{
            font-family: {ZH_FONT};
            font-weight: 900;
            font-size: 14px;
            background: {COLORS["panel"]};
            padding: 3px 10px;
            border-bottom: 1px solid {COLORS["border"]};
            color: {COLORS["text"]};
        }}
        QMenuBar::item {{
            padding: 7px 14px;
            border-radius: 4px;
        }}
        QMenuBar::item:selected {{
            background-color: #3B4048;
            color: {COLORS["white"]};
        }}
        QStatusBar {{
            font-family: {MIXED_FONT};
            font-weight: 900;
            font-size: 14px;
            background: {COLORS["panel"]};
            border-top: 1px solid {COLORS["border"]};
            color: {COLORS["primary"]};
        }}
        QMainWindow::separator {{
            background: {COLORS["border_alt"]};
            width: 8px;
            height: 8px;
        }}
        QMainWindow::separator:hover {{
            background: {COLORS["primary"]};
        }}
        QGroupBox {{
            background-color: {COLORS["panel"]};
            border: 1px solid {COLORS["border"]};
            border-radius: 6px;
            margin-top: 0px;
        }}
        QGroupBox#metricCard {{
            background-color: {COLORS["panel_alt"]};
            border: 1px solid {COLORS["border_alt"]};
            border-radius: 8px;
        }}
        QLabel#waveTitle {{
            color: {COLORS["primary"]};
            font-size: 24px;
            font-family: {MONO_FONT};
            font-weight: 900;
            padding-left: 4px;
        }}
        QLabel#waveLabel {{
            background-color: {COLORS["surface"]};
            border: 1px solid {COLORS["border"]};
            border-radius: 2px;
        }}
        QLabel#metricName {{
            color: {COLORS["text_dim"]};
            font-size: 24px;
            font-family: {ZH_FONT};
            font-weight: 900;
        }}
        QLabel#metricUnit {{
            color: {COLORS["text_muted"]};
            font-size: 24px;
            font-family: {MONO_FONT};
            font-weight: 900;
        }}
        QLabel#metricValue {{
            font-size: 50px;
            font-family: {MONO_FONT};
            font-weight: 900;
        }}
        QLabel#statusText {{
            {status_label_style(False)}
            font-size: 20px;
            font-family: {ZH_FONT};
            font-weight: 900;
        }}
    """


def menu_bar_style():
    return f"""
        QMenuBar {{
            background-color: {COLORS["panel"]};
            color: {COLORS["text"]};
            border-bottom: 1px solid {COLORS["border"]};
            font-family: {ZH_FONT};
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
            color: {COLORS["white"]};
        }}
    """


def status_bar_style():
    return f"""
        QStatusBar {{
            background-color: {COLORS["panel"]};
            color: {COLORS["primary"]};
            border-top: 1px solid {COLORS["border"]};
            font-family: {ZH_FONT};
            font-size: 16px;
            font-weight: 900;
        }}
    """


def wave_label_style():
    return f"""
        background-color: {COLORS["surface"]};
        border: 1px solid {COLORS["border"]};
        border-radius: 2px;
    """


def metric_card_style():
    return f"""
        QGroupBox {{
            background-color: {COLORS["panel_alt"]};
            border: 1px solid {COLORS["border_alt"]};
            border-radius: 8px;
            margin-top: 0px;
        }}
    """


def metric_value_style(color):
    return f"color: {color}; background: transparent;"


def metric_name_style(color=None):
    text_color = color or COLORS["text_dim"]
    return f"""
        color: {text_color};
        background: transparent;
        font-size: 24px;
        font-family: {ZH_FONT};
        font-weight: 900;
    """


def metric_unit_style():
    return f"""
        color: {COLORS["text_muted"]};
        background: transparent;
        font-size: 24px;
        font-family: {MONO_FONT};
        font-weight: 900;
    """


def wave_title_style(color):
    return f"""
        color: {color};
        background: transparent;
        font-size: 24px;
        font-family: {MONO_FONT};
        font-weight: 900;
        padding-left: 6px;
    """


def status_label_style(alarm):
    color = COLORS["alarm"] if alarm else COLORS["ecg"]
    background = COLORS["alarm_bg"] if alarm else COLORS["normal_bg"]
    return (
        f"color: {color}; "
        f"background-color: {background}; "
        f"border: 1px solid {color}; "
        "border-radius: 6px; "
        "padding: 6px 10px;"
    )


def alarm_title_style(alarm):
    color = COLORS["alarm"] if alarm else COLORS["ecg"]
    return f"color: {color}; font-size: 22px; font-family: {ZH_FONT}; font-weight: 900;"


def debug_dock_style():
    return f"""
        QDockWidget {{
            background: {COLORS["panel"]};
            color: {COLORS["text"]};
            border: 1px solid {COLORS["border"]};
            font-family: {ZH_FONT};
            font-size: 14px;
            font-weight: 900;
        }}
        QDockWidget::title {{
            background: {COLORS["panel"]};
            color: {COLORS["text"]};
            padding: 6px 8px;
            border-bottom: 1px solid {COLORS["border"]};
            text-align: left;
        }}
        QDockWidget::close-button,
        QDockWidget::float-button {{
            background: {COLORS["button"]};
            border: 1px solid {COLORS["border"]};
            border-radius: 2px;
            width: 14px;
            height: 14px;
        }}
        QDockWidget::close-button:hover,
        QDockWidget::float-button:hover {{
            background: {COLORS["button_checked"]};
            border-color: {COLORS["primary"]};
        }}
    """


def debug_text_style():
    return f"""
        QPlainTextEdit {{
            background-color: {COLORS["surface"]};
            color: {COLORS["text"]};
            border: 1px solid {COLORS["border"]};
            font-family: {MONO_FONT};
            font-size: 12px;
        }}
    """


def debug_controls_style():
    return f"""
        QGroupBox {{
            background-color: {COLORS["panel_alt"]};
            border: 1px solid {COLORS["border"]};
            border-radius: 6px;
            margin-top: 8px;
            color: {COLORS["text_muted"]};
            font-family: {MONO_FONT};
            font-size: 12px;
            font-weight: 900;
        }}
        QGroupBox::title {{
            subcontrol-origin: margin;
            left: 10px;
            padding: 0 4px;
            color: {COLORS["text"]};
        }}
        QPushButton {{
            background-color: {COLORS["button"]};
            color: {COLORS["white"]};
            border: 1px solid {COLORS["border"]};
            border-radius: 6px;
            padding: 7px 10px;
            font-family: {ZH_FONT};
            font-size: 13px;
            font-weight: 900;
        }}
        QPushButton:hover {{
            background-color: {COLORS["button_checked"]};
            border-color: {COLORS["primary"]};
        }}
        QPushButton:checked {{
            background-color: {COLORS["primary"]};
            border-color: {COLORS["primary"]};
            color: {COLORS["white"]};
        }}
        QPushButton#clearDebugButton {{
            background-color: #C678DD;
            border-color: #C678DD;
        }}
        QPushButton#clearDebugButton:hover {{
            background-color: #B560D0;
            border-color: #B560D0;
        }}
        QPushButton#errorOnlyButton:checked {{
            background-color: {COLORS["alarm"]};
            border-color: {COLORS["alarm"]};
        }}
    """


def uart_dialog_style():
    return f"""
        QWidget {{
            background-color: {COLORS["window"]};
            color: {COLORS["text"]};
            font-weight: 900;
        }}
        QLabel {{
            background: transparent;
        }}
        QComboBox {{
            background-color: {COLORS["panel_alt"]};
            border: 1px solid {COLORS["border"]};
            border-radius: 6px;
            padding: 5px 10px;
            color: {COLORS["text"]};
        }}
        QPushButton {{
            background-color: {COLORS["button"]};
            border: 1px solid {COLORS["primary"]};
            border-radius: 6px;
            padding: 7px 12px;
            color: {COLORS["white"]};
        }}
        QPushButton:hover {{
            background-color: {COLORS["button_checked"]};
        }}
        QPushButton:disabled {{
            color: {COLORS["text_dim"]};
            border-color: {COLORS["border"]};
        }}
        ComboBox, PushButton {{
            background-color: {COLORS["panel_alt"]};
            color: {COLORS["text"]};
            border: 1px solid {COLORS["border"]};
            border-radius: 6px;
        }}
    """
