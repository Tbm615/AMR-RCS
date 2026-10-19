"""Dark theme."""
from __future__ import annotations

import os
import tempfile

BG = "#0f1115"          # window
PANEL = "#151923"       # group boxes / cards
FIELD = "#1b2030"       # inputs
BORDER = "#262c3a"
TEXT = "#e6e8ee"
MUTED = "#8b93a7"
ACCENT = "#3b82f6"

QSS = """
* {{ font-family: 'Inter','Segoe UI','Ubuntu','Noto Sans',sans-serif; font-size: 13px; color: {TEXT}; }}

/* Only top-level surfaces paint a background; everything else is
   transparent so labels/checkboxes don't draw dark patches on cards. */
QWidget {{ background: transparent; }}
QMainWindow, QDialog, QStackedWidget, QMessageBox {{ background: {BG}; }}
QAbstractItemView, QAbstractScrollArea {{ background: {BG}; }}

QLabel[muted="true"] {{ color: {MUTED}; }}
QLabel[danger="true"] {{ color: #ff6a6a; font-weight: 600; }}
QLabel[success="true"] {{ color: #39d98a; font-weight: 600; }}
QLabel[warn="true"] {{ color: #ffb547; font-weight: 600; }}
QLabel#PageTitle {{ font-size: 18px; font-weight: 600; }}
QLabel#Hint {{ color: {MUTED}; }}

QFrame#Sidebar {{ background: #0b0d12; border-right: 1px solid #1a1d26; }}
QPushButton#NavButton {{
    background: transparent; color: #c6cad6; text-align: left;
    padding: 10px 16px; border: none; border-left: 3px solid transparent; border-radius: 0;
}}
QPushButton#NavButton:hover {{ background: #141821; color: #ffffff; }}
QPushButton#NavButton:checked {{ background: #161b27; color: #ffffff; border-left: 3px solid {ACCENT}; }}

QPushButton {{
    background: #1f2533; border: 1px solid {BORDER}; border-radius: 6px; padding: 7px 14px;
}}
QPushButton:hover {{ background: #283044; }}
QPushButton:pressed {{ background: #1a1f2b; }}
QPushButton:disabled {{ color: #5b6378; background: #161a24; }}
QPushButton[kind="primary"] {{ background: #2563eb; border-color: #3b82f6; color: white; }}
QPushButton[kind="primary"]:hover {{ background: #2f6ff0; }}
QPushButton[kind="danger"] {{ background: #c62828; border-color: #e53935; color: white; }}
QPushButton[kind="danger"]:hover {{ background: #d53030; }}
QPushButton[kind="danger"]:checked {{ background: #7f1d1d; border: 2px solid #ff6a6a; }}
QPushButton[kind="success"] {{ background: #2e7d32; border-color: #43a047; color: white; }}
QPushButton[kind="success"]:hover {{ background: #358a3a; }}

QLineEdit, QAbstractSpinBox, QComboBox, QTextEdit, QPlainTextEdit {{
    background: {FIELD}; border: 1px solid {BORDER}; border-radius: 5px;
    padding: 5px 8px; selection-background-color: #2563eb;
}}
QLineEdit:focus, QAbstractSpinBox:focus, QComboBox:focus {{ border-color: {ACCENT}; }}
QComboBox::drop-down {{ border: none; width: 22px; }}
QComboBox::down-arrow {{ image: url("{DOWN}"); width: 10px; height: 6px; }}
QComboBox QAbstractItemView {{ background: {FIELD}; border: 1px solid {BORDER}; selection-background-color: #24304a; }}
QAbstractSpinBox {{ padding-right: 22px; }}
QAbstractSpinBox::up-button, QAbstractSpinBox::down-button {{
    subcontrol-origin: border; width: 20px; border: none; background: transparent;
}}
QAbstractSpinBox::up-button {{ subcontrol-position: top right; }}
QAbstractSpinBox::down-button {{ subcontrol-position: bottom right; }}
QAbstractSpinBox::up-button:hover, QAbstractSpinBox::down-button:hover {{ background: #283044; }}
QAbstractSpinBox::up-arrow {{ image: url("{UP}"); width: 8px; height: 5px; }}
QAbstractSpinBox::down-arrow {{ image: url("{DOWN}"); width: 8px; height: 5px; }}

QListWidget, QTableWidget {{ background: {FIELD}; border: 1px solid {BORDER}; border-radius: 5px; }}
QListWidget::item {{ padding: 6px 8px; }}
QListWidget::item:selected, QTableWidget::item:selected {{ background: #24304a; color: #ffffff; }}
QHeaderView::section {{ background: #1f2533; color: {MUTED}; border: none; border-bottom: 1px solid {BORDER}; padding: 5px; }}
QTableWidget {{ gridline-color: {BORDER}; }}
QTableCornerButton::section {{ background: #1f2533; border: none; }}

QGroupBox {{
    background: {PANEL}; border: 1px solid #1f2533; border-radius: 8px;
    margin-top: 22px; padding: 12px 10px 10px 10px;
}}
QGroupBox::title {{
    subcontrol-origin: margin; subcontrol-position: top left; left: 4px; top: 2px;
    padding: 0 4px; color: {MUTED}; font-weight: 600;
}}

QFrame#Card {{ background: {PANEL}; border: 1px solid #1f2533; border-radius: 8px; }}

QStatusBar {{ background: #0b0d12; border-top: 1px solid #1a1d26; color: #c6cad6; }}
QStatusBar QLabel {{ color: #c6cad6; }}
QToolTip {{ background: #1f2533; border: 1px solid {BORDER}; padding: 4px; }}

QScrollBar:vertical {{ background: transparent; width: 10px; margin: 0; }}
QScrollBar::handle:vertical {{ background: #2a3142; border-radius: 5px; min-height: 24px; }}
QScrollBar::handle:vertical:hover {{ background: #3a4256; }}
QScrollBar::add-line, QScrollBar::sub-line {{ background: none; height: 0; width: 0; }}
QScrollBar:horizontal {{ background: transparent; height: 10px; margin: 0; }}
QScrollBar::handle:horizontal {{ background: #2a3142; border-radius: 5px; min-width: 24px; }}

QCheckBox {{ spacing: 8px; }}
QCheckBox::indicator {{ width: 16px; height: 16px; border: 1px solid #3a4256; background: {FIELD}; border-radius: 4px; }}
QCheckBox::indicator:checked {{ background: {ACCENT}; border-color: {ACCENT}; image: url("{CHECK}"); }}
QRadioButton {{ spacing: 8px; }}
QRadioButton::indicator {{ width: 14px; height: 14px; border: 1px solid #3a4256; background: {FIELD}; border-radius: 8px; }}
QRadioButton::indicator:checked {{ border-color: {ACCENT};
    background: qradialgradient(cx:0.5, cy:0.5, radius:0.5, fx:0.5, fy:0.5,
                                stop:0 {ACCENT}, stop:0.5 {ACCENT}, stop:0.6 {FIELD}, stop:1 {FIELD}); }}
QPushButton[kind="primary"]:disabled, QPushButton[kind="success"]:disabled,
QPushButton[kind="danger"]:disabled {{ color: #5b6378; background: #161a24; border-color: {BORDER}; }}
"""


def _arrow_images() -> dict:
    """Draw the small arrow/check icons the stylesheet needs into PNG files.
    Done at runtime with QPainter so there is no dependency on the SVG
    image plugin and nothing extra to bundle."""
    from PySide6.QtCore import QPointF, Qt
    from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPolygonF

    out = os.path.join(tempfile.gettempdir(), "rcs_theme")
    os.makedirs(out, exist_ok=True)

    def tri(name, pts, w, h, color):
        img = QImage(w * 2, h * 2, QImage.Format_ARGB32)   # 2x for HiDPI
        img.fill(Qt.transparent)
        p = QPainter(img)
        p.setRenderHint(QPainter.Antialiasing)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(color))
        p.drawPolygon(QPolygonF([QPointF(x * 2, y * 2) for x, y in pts]))
        p.end()
        path = os.path.join(out, name)
        img.save(path)
        return path.replace("\\", "/")

    def check(name):
        img = QImage(32, 32, QImage.Format_ARGB32)
        img.fill(Qt.transparent)
        p = QPainter(img)
        p.setRenderHint(QPainter.Antialiasing)
        p.setPen(QPen(QColor("white"), 4, Qt.SolidLine, Qt.RoundCap, Qt.RoundJoin))
        p.drawPolyline([QPointF(8, 17), QPointF(14, 23), QPointF(25, 10)])
        p.end()
        path = os.path.join(out, name)
        img.save(path)
        return path.replace("\\", "/")

    return {
        "DOWN": tri("down.png", [(0, 0), (10, 0), (5, 6)], 10, 6, MUTED),
        "UP": tri("up.png", [(0, 6), (10, 6), (5, 0)], 10, 6, MUTED),
        "CHECK": check("check.png"),
    }


def apply(app) -> None:
    imgs = _arrow_images()
    app.setStyleSheet(QSS.format(TEXT=TEXT, MUTED=MUTED, ACCENT=ACCENT, BG=BG,
                                 PANEL=PANEL, FIELD=FIELD, BORDER=BORDER, **imgs))
