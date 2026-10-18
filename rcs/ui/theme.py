"""Dark theme stylesheet."""

QSS = """
* { font-family: 'Inter','Segoe UI','Ubuntu',sans-serif; font-size: 13px; }
QMainWindow, QWidget { background: #0f1115; color: #e6e8ee; }
QLabel { color: #e6e8ee; }
QLabel[muted="true"] { color: #8b93a7; }
QLabel[danger="true"] { color: #ff6a6a; font-weight: 600; }
QLabel[success="true"] { color: #39d98a; font-weight: 600; }
QLabel[warn="true"] { color: #ffb547; font-weight: 600; }

QFrame#Sidebar { background: #0b0d12; border-right: 1px solid #1a1d26; }
QPushButton#NavButton {
    background: transparent; color: #c6cad6; text-align: left;
    padding: 10px 14px; border: none; border-left: 3px solid transparent;
    font-size: 13px;
}
QPushButton#NavButton:hover { background: #141821; color: #ffffff; }
QPushButton#NavButton:checked {
    background: #141821; color: #ffffff; border-left: 3px solid #5a9df6;
}

QPushButton {
    background: #1a1f2b; color: #e6e8ee; border: 1px solid #262c3a;
    border-radius: 6px; padding: 7px 14px;
}
QPushButton:hover { background: #222a3a; }
QPushButton:pressed { background: #171c27; }
QPushButton:disabled { color: #5b6378; background: #141821; }
QPushButton[kind="primary"] { background: #2a66d4; border: 1px solid #3a79e8; color: white; }
QPushButton[kind="primary"]:hover { background: #3374df; }
QPushButton[kind="danger"] { background: #c62828; border: 1px solid #e53935; color: white; }
QPushButton[kind="danger"]:hover { background: #d53030; }
QPushButton[kind="success"] { background: #2e7d32; border: 1px solid #43a047; color: white; }

QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox, QTextEdit, QPlainTextEdit {
    background: #141821; border: 1px solid #262c3a; border-radius: 5px;
    padding: 5px 8px; color: #e6e8ee; selection-background-color: #2a66d4;
}
QComboBox::drop-down { border: none; width: 20px; }
QComboBox QAbstractItemView { background: #141821; border: 1px solid #262c3a; }

QListWidget, QTreeWidget, QTableWidget {
    background: #0f1115; border: 1px solid #1a1d26; alternate-background-color: #121520;
}
QListWidget::item:selected, QTreeWidget::item:selected, QTableWidget::item:selected {
    background: #1e2b48; color: #ffffff;
}

QGroupBox {
    border: 1px solid #1a1d26; border-radius: 6px; margin-top: 14px;
    padding: 10px; background: #121520;
}
QGroupBox::title {
    subcontrol-origin: margin; left: 10px; padding: 0 6px; color: #8b93a7;
}

QStatusBar { background: #0b0d12; border-top: 1px solid #1a1d26; color: #c6cad6; }
QToolTip { background: #1a1f2b; color: #e6e8ee; border: 1px solid #262c3a; }

QScrollBar:vertical { background: #0f1115; width: 10px; margin: 0; }
QScrollBar::handle:vertical { background: #262c3a; border-radius: 5px; min-height: 20px; }
QScrollBar::handle:vertical:hover { background: #3a4256; }
QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { background: none; height: 0; }
QScrollBar:horizontal { background: #0f1115; height: 10px; margin: 0; }
QScrollBar::handle:horizontal { background: #262c3a; border-radius: 5px; min-width: 20px; }

QTabWidget::pane { border: 1px solid #1a1d26; background: #0f1115; }
QTabBar::tab {
    background: #0b0d12; color: #8b93a7; padding: 7px 14px;
    border: 1px solid #1a1d26; border-bottom: none;
}
QTabBar::tab:selected { background: #141821; color: #e6e8ee; }

QCheckBox { color: #e6e8ee; }
QCheckBox::indicator { width: 16px; height: 16px; border: 1px solid #262c3a; background: #141821; border-radius: 3px; }
QCheckBox::indicator:checked { background: #2a66d4; border-color: #3a79e8; }
"""


def apply(app) -> None:
    app.setStyleSheet(QSS)
