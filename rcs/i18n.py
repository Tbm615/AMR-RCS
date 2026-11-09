"""The app in another language (English or Vietnamese), chosen on the Connection
page and applied when the app starts.

The pages are written in English. install("vi") wraps the Qt calls that put
text on screen (constructors of labels, buttons, check boxes, group boxes;
setText, setToolTip, setTitle, setWindowTitle, setPlaceholderText, addItem,
addTab, status bar messages, header labels, message boxes) so each text goes
through tr(): an exact match in the catalog, else a pattern for texts with a
number or a name in them, else the English as it is. Nothing else changes,
and English needs no wrapping at all.
"""
from __future__ import annotations

import re
from typing import Callable, Dict, List, Tuple, Union

from .i18n_vi import PATTERNS as VI_PATTERNS
from .i18n_vi import TEXTS as VI_TEXTS

LANGS = {"en": "English", "vi": "Tiếng Việt"}
_TAG = re.compile(r"(<[^>]+>)")
_texts: Dict[str, str] = {}
_patterns: List[Tuple["re.Pattern", Union[str, Callable]]] = []
current = "en"
_installed = False


def tr(text):
    """The text in the chosen language (English: as it is)."""
    if not isinstance(text, str) or not _texts or not text:
        return text
    hit = _texts.get(text)
    if hit is not None:
        return hit
    stripped = text.strip()
    if stripped != text and stripped in _texts:
        return text.replace(stripped, _texts[stripped])
    for rx, rep in _patterns:
        m = rx.fullmatch(text)
        if m:
            return rep(m) if callable(rep) else m.expand(rep)
    if "<" in text and ">" in text:            # rich text: the words between the tags
        parts = _TAG.split(text)
        out = [p if (i % 2 or not p.strip()) else tr(p) for i, p in enumerate(parts)]
        joined = "".join(out)
        if joined != text:
            return joined
    head, sep, rest = text.partition(": ")      # "Source: what happened" (status bar)
    if sep and len(head) < 30:
        a, b = tr(head), tr(rest)
        if (a, b) != (head, rest):
            return f"{a}: {b}"
    return text


def set_language(lang: str) -> None:
    global current, _texts, _patterns
    current = lang if lang in LANGS else "en"
    if current == "vi":
        _texts = dict(VI_TEXTS)
        _patterns = [(re.compile(p, re.S), r) for p, r in VI_PATTERNS]
    else:
        _texts, _patterns = {}, []


def install(lang: str) -> None:
    """Choose the language and wrap Qt's text calls (once, before the window
    is built)."""
    set_language(lang)
    if current == "en" or _installed:
        return
    _wrap()


def _first(args):
    return (tr(args[0]),) + tuple(args[1:]) if args and isinstance(args[0], str) else args


def _wrap() -> None:
    global _installed
    from PySide6 import QtWidgets as W
    from PySide6.QtGui import QAction

    def ctor(cls):
        orig = cls.__init__

        def init(self, *a, **k):
            orig(self, *_first(a), **k)
        cls.__init__ = init

    for cls in (W.QLabel, W.QPushButton, W.QCheckBox, W.QRadioButton, W.QGroupBox, W.QToolButton, QAction, W.QMenu):
        ctor(cls)

    def method(cls, name, which=0):
        orig = getattr(cls, name)

        def call(self, *a, **k):
            a = list(a)
            if len(a) > which and isinstance(a[which], str):
                a[which] = tr(a[which])
            return orig(self, *a, **k)
        setattr(cls, name, call)

    method(W.QLabel, "setText")
    method(W.QAbstractButton, "setText")
    method(W.QWidget, "setToolTip")
    method(W.QWidget, "setWindowTitle")
    method(W.QGroupBox, "setTitle")
    method(W.QLineEdit, "setPlaceholderText")
    method(W.QStatusBar, "showMessage")
    method(QAction, "setText")
    method(W.QMenu, "setTitle")
    method(W.QTabWidget, "setTabText", 1)
    method(W.QFormLayout, "addRow")             # addRow("label", widget) makes its own QLabel
    method(W.QFormLayout, "insertRow", 1)
    method(W.QTabBar, "setTabText", 1)

    for cls in (W.QTabWidget,):
        orig_add = cls.addTab

        def add_tab(self, widget, *a, _o=orig_add):
            a = list(a)
            if a and isinstance(a[-1], str):
                a[-1] = tr(a[-1])
            return _o(self, widget, *a)
        cls.addTab = add_tab

    orig_item = W.QComboBox.addItem

    def add_item(self, *a, **k):
        a = list(a)
        for i, v in enumerate(a[:2]):         # (text, data) or (icon, text, data)
            if isinstance(v, str):
                a[i] = tr(v)
                break
        return orig_item(self, *a, **k)
    W.QComboBox.addItem = add_item
    orig_items = W.QComboBox.addItems
    W.QComboBox.addItems = lambda self, items: orig_items(self, [tr(x) for x in items])

    for cls in (W.QTableWidget,):
        orig_h = cls.setHorizontalHeaderLabels
        cls.setHorizontalHeaderLabels = lambda self, labels, _o=orig_h: _o(self, [tr(x) for x in labels])

    for name in ("question", "information", "warning", "critical"):
        orig_box = getattr(W.QMessageBox, name)

        def box(parent, title, text, *a, _o=orig_box, **k):
            return _o(parent, tr(title), tr(text), *a, **k)
        setattr(W.QMessageBox, name, staticmethod(box))

    orig_text = W.QInputDialog.getText
    W.QInputDialog.getText = staticmethod(lambda parent, title, label, *a, _o=orig_text, **k:
                                          _o(parent, tr(title), tr(label), *a, **k))
    orig_get_item = W.QInputDialog.getItem
    W.QInputDialog.getItem = staticmethod(lambda parent, title, label, *a, _o=orig_get_item, **k:
                                          _o(parent, tr(title), tr(label), *a, **k))
    for name in ("getSaveFileName", "getOpenFileName"):
        orig_fd = getattr(W.QFileDialog, name)
        setattr(W.QFileDialog, name, staticmethod(lambda parent, caption="", *a, _o=orig_fd, **k:
                                                  _o(parent, tr(caption), *a, **k)))
    _installed = True
