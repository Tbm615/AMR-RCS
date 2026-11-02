"""Page workspace: show several pages at once, side by side, as tabs or in
their own window.

The sidebar still works like before — a click shows that page in place of
the one you were using — but a page can also be opened beside or below the
one in use, as a tab, or in a separate window (Ctrl+click or right-click a
sidebar entry, or right-click a tab), and preset layouts put the usual
combinations side by side (Teleop + 2D Map, 2D Map + 3D Map, …).

Built from QSplitter + QTabWidget rather than QDockWidget: splitting a
dock that has ever been in a tab group throws Qt's dock layout off screen.
The widget tree is the layout; save_layout() walks it into plain JSON.

The main window asks visible_keys() which pages to redraw and which heavy
streams (camera, 3D) to subscribe, so a page that is tabbed away or closed
costs nothing, exactly like a hidden page did before.

Pages never leave the main window except into a page window: a closed page
is parked in a hidden child. Moving a QOpenGLWidget to another top-level
window recreates its GL context, and pyqtgraph's GL items keep buffers and
shaders from the old one — so pages listed in `no_float` (the 3D Map) are
never put in a separate window.
"""
from __future__ import annotations

from typing import Dict, List, Optional

from PySide6.QtCore import QEvent, QObject, Qt, QTimer, Signal
from PySide6.QtWidgets import (
    QApplication, QMenu, QSplitter, QTabWidget, QVBoxLayout, QWidget,
)

# preset name -> menu label (layouts are built in Workspace.apply_preset)
PRESETS = [
    ("single", "Single page"),
    ("drive", "Drive — Teleop + 2D Map"),
    ("mapping", "Mapping — 2D Map + 3D Map"),
    ("monitor", "Monitor — 2D Map + Camera + Status"),
    ("missions", "Missions — Missions + Camera"),
]


class _Pane(QTabWidget):
    """A group of pages shown as tabs; one pane per split region or window."""

    def __init__(self, ws: "Workspace", parent=None):
        super().__init__(parent)
        self.ws = ws
        self.setDocumentMode(True)
        self.setMovable(True)
        self.setTabsClosable(True)
        self.tabCloseRequested.connect(lambda i: self.ws.close_page(self.widget(i).property("page_key")))
        self.currentChanged.connect(self._on_current)
        self.tabBar().setContextMenuPolicy(Qt.CustomContextMenu)
        self.tabBar().customContextMenuRequested.connect(self._tab_menu)

    def keys(self) -> List[str]:
        return [self.widget(i).property("page_key") for i in range(self.count())]

    def current_key(self) -> str:
        w = self.currentWidget()
        return w.property("page_key") if w is not None else ""

    def _on_current(self, _i: int) -> None:
        k = self.current_key()
        if k and self.isVisible():
            self.ws._set_active(k)
        self.ws._schedule_emit()

    def _tab_menu(self, pos) -> None:
        i = self.tabBar().tabAt(pos)
        if i < 0:
            return
        k = self.widget(i).property("page_key")
        self.ws.page_menu(k).exec(self.tabBar().mapToGlobal(pos))


class _FloatWindow(QWidget):
    """A page in its own top-level window (e.g. on a second monitor)."""

    def __init__(self, ws: "Workspace"):
        super().__init__(None, Qt.Window)
        self.ws = ws
        self.pane = _Pane(ws, self)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        lay.addWidget(self.pane)
        self.pane.currentChanged.connect(self._retitle)

    def _retitle(self, _i=None) -> None:
        k = self.pane.current_key()
        self.setWindowTitle(f"RCS — {self.ws.titles.get(k, k)}")

    def closeEvent(self, ev) -> None:
        # closing the window closes its pages (the sidebar reopens them)
        if not self.ws._rebuilding:
            for k in self.pane.keys():
                self.ws._take(k)
            self.ws._forget_float(self)
        super().closeEvent(ev)


class Workspace(QWidget):
    visible_changed = Signal()          # some page became visible or hidden
    active_changed = Signal(str)        # key of the page the operator is using

    def __init__(self, parent=None):
        super().__init__(parent)
        self.pages: Dict[str, QWidget] = {}
        self.titles: Dict[str, str] = {}
        self.order: List[str] = []
        self.active: str = ""
        self.floats: List[_FloatWindow] = []
        self.no_float: set = set()          # pages that must stay in the main window
        self._park = QWidget(self)          # closed pages live here, hidden
        self._park.hide()
        self._emit_pending = False
        self._rebuilding = False
        lay = QVBoxLayout(self)
        lay.setContentsMargins(0, 0, 0, 0)
        self.root = self._splitter(Qt.Horizontal)
        lay.addWidget(self.root)
        QApplication.instance().installEventFilter(self)
        QApplication.instance().focusChanged.connect(self._on_focus)

    # ---- pages ----

    def add_page(self, key: str, title: str, widget: QWidget) -> None:
        widget.setProperty("page_key", key)
        self.pages[key] = widget
        self.titles[key] = title
        self.order.append(key)
        widget.hide()
        widget.setParent(self._park)

    def visible_keys(self) -> set:
        return {k for k, w in self.pages.items() if w.isVisible()}

    def open_keys(self) -> set:
        """Pages that are open somewhere, possibly behind another tab."""
        return {k for p in self._panes() for k in p.keys()}

    def show_page(self, key: str, where: str = "replace") -> None:
        """where: 'replace' (in place of the page in use), 'tab', 'right',
        'below' or 'float'."""
        if key not in self.pages:
            return
        pane = self._pane_of(key)
        if pane is not None and where in ("replace", "tab"):
            pane.setCurrentWidget(self.pages[key])        # already open: bring forward
            pane.window().raise_()
            self._set_active(key)
            self._schedule_emit()
            return
        if where == "float" and key in self.no_float:
            where = "right"
        anchor = self._anchor(exclude=key)
        if pane is not None:
            self._take(key)
            if anchor is not None and anchor.count() == 0:
                anchor = self._anchor(exclude=key)
        if where == "float":
            self._float([key], self.window().geometry())
        elif anchor is None:
            p = self._root_pane()
            self._add_tab(p, key)
        elif where in ("replace", "tab"):
            idx = anchor.currentIndex()
            old = anchor.current_key()
            self._add_tab(anchor, key, idx + 1)
            if where == "replace" and old and old != key:
                self._take(old)
        else:
            self._split(anchor, key, Qt.Horizontal if where == "right" else Qt.Vertical)
        self._set_active(key)
        self._update_tabbars()
        self._schedule_emit()

    def close_page(self, key: str) -> None:
        if self._pane_of(key) is None:
            return
        self._take(key)
        if key == self.active:
            self._pick_active()
        self._update_tabbars()
        self._schedule_emit()

    def page_menu(self, key: str) -> QMenu:
        m = QMenu(self)
        m.addAction("Show here", lambda: self.show_page(key, "replace"))
        m.addAction("Open as a tab", lambda: self.show_page(key, "tab"))
        m.addAction("Open beside (right)", lambda: self.show_page(key, "right"))
        m.addAction("Open below", lambda: self.show_page(key, "below"))
        a = m.addAction("Open in a new window", lambda: self.show_page(key, "float"))
        if key in self.no_float:
            a.setEnabled(False)
            a.setText("Open in a new window (not for this page yet)")
        pane = self._pane_of(key)
        if pane is not None and isinstance(pane.window(), _FloatWindow):
            m.addAction("Move back to the main window", lambda: self._dock_back(key))
        if pane is not None:
            m.addSeparator()
            m.addAction("Close", lambda: self.close_page(key))
        return m

    # ---- presets ----

    def apply_preset(self, name: str, single_key: str = "conn") -> None:
        h, v = "h", "v"
        tabs = lambda *ks: {"tabs": list(ks), "current": ks[0]}          # noqa: E731
        split = lambda o, sizes, *ch: {"split": o, "sizes": list(sizes), "children": list(ch)}  # noqa: E731
        if name == "drive":
            tree, act = split(h, [2, 3], tabs("teleop"), tabs("map")), "map"
        elif name == "mapping":
            tree, act = split(h, [1, 1], tabs("map"), tabs("3d")), "map"
        elif name == "monitor":
            tree, act = split(h, [3, 2], tabs("map"), split(v, [1, 1], tabs("cam"), tabs("status"))), "map"
        elif name == "missions":
            tree, act = split(h, [3, 2], tabs("mission"), tabs("cam")), "mission"
        else:
            k = single_key if single_key in self.pages else self.order[0]
            tree, act = tabs(k), k
        self._build({"tree": tree, "floats": [], "active": act})

    # ---- persistence ----

    def save_layout(self) -> dict:
        return {
            "tree": self._dump(self.root),
            "floats": [{"tabs": f.pane.keys(), "current": f.pane.current_key(),
                        "geo": list(f.geometry().getRect())} for f in self.floats],
            "active": self.active,
        }

    def restore_layout(self, data: dict) -> bool:
        try:
            ok = self._build(data)
        except (KeyError, TypeError, ValueError, AttributeError):
            ok = False
        return ok

    # ---- building ----

    def _splitter(self, orient) -> QSplitter:
        s = QSplitter(orient)
        s.setChildrenCollapsible(False)
        s.setHandleWidth(5)
        return s

    def _build(self, data: dict) -> bool:
        seen: set = set()

        def valid(node) -> bool:
            if "tabs" in node:
                ks = [k for k in node["tabs"] if k in self.pages and k not in seen]
                seen.update(ks)
                return bool(ks)
            return any([valid(c) for c in node.get("children", [])])

        tree = data.get("tree") or {}
        floats = [f for f in data.get("floats", []) if valid(f)]
        if not valid(tree) and not floats:
            return False
        self._rebuilding = True
        try:
            for k in list(self.pages):
                self._detach(k)
            for f in list(self.floats):
                f.close(); f.deleteLater()
            self.floats = []
            old = self.root
            used: set = set()
            self.root = self._make(tree, used, top=True)
            self.layout().removeWidget(old)
            old.setParent(None)
            old.deleteLater()
            self.layout().addWidget(self.root)
            for f in floats:
                ks = [k for k in f["tabs"] if k in self.pages and k not in used and k not in self.no_float]
                if ks:
                    used.update(ks)
                    fw = self._float(ks, None, f.get("current"))
                    g = f.get("geo")
                    if g and len(g) == 4:
                        fw.setGeometry(*[int(x) for x in g])
        finally:
            self._rebuilding = False
        act = data.get("active", "")
        self.active = ""
        self._set_active(act if self._pane_of(act) else next(iter(self.open_keys_ordered()), ""))
        self._update_tabbars()
        self._schedule_emit()
        return True

    def _make(self, node, used: set, top: bool = False) -> QWidget:
        if "tabs" in node:
            p = _Pane(self)
            for k in node["tabs"]:
                if k in self.pages and k not in used:
                    used.add(k)
                    self._add_tab(p, k)
            cur = node.get("current")
            if cur in self.pages and self._pane_of(cur) is p:
                p.setCurrentWidget(self.pages[cur])
            if top:     # the root is always a splitter
                s = self._splitter(Qt.Horizontal)
                s.addWidget(p)
                return s
            return p
        s = self._splitter(Qt.Horizontal if node.get("split") == "h" else Qt.Vertical)
        for c in node.get("children", []):
            w = self._make(c, used)
            if isinstance(w, _Pane) and w.count() == 0:
                w.deleteLater()
                continue
            s.addWidget(w)
        sizes = node.get("sizes")
        if sizes and len(sizes) == s.count():
            total = sum(sizes) or 1
            span = 1000
            s.setSizes([int(span * x / total) for x in sizes])
        if top and s.count() == 0:
            s.addWidget(_Pane(self))
        return s

    def _dump(self, w) -> dict:
        if isinstance(w, _Pane):
            return {"tabs": w.keys(), "current": w.current_key()}
        kids = [w.widget(i) for i in range(w.count())]
        if len(kids) == 1:
            return self._dump(kids[0])
        return {"split": "h" if w.orientation() == Qt.Horizontal else "v",
                "sizes": w.sizes(), "children": [self._dump(c) for c in kids]}

    # ---- tree edits ----

    def _panes(self) -> List[_Pane]:
        out = list(self.root.findChildren(_Pane))
        for f in self.floats:
            out.append(f.pane)
        return out

    def open_keys_ordered(self) -> List[str]:
        return [k for p in self._panes() for k in p.keys()]

    def _pane_of(self, key: str) -> Optional[_Pane]:
        w = self.pages.get(key)
        for p in self._panes():
            if w is not None and p.indexOf(w) >= 0:
                return p
        return None

    def _root_pane(self) -> _Pane:
        panes = self.root.findChildren(_Pane)
        if panes:
            return panes[0]
        p = _Pane(self)
        self.root.addWidget(p)
        return p

    def _anchor(self, exclude: str = "") -> Optional[_Pane]:
        """The pane of the page in use, else the first non-empty main pane."""
        p = self._pane_of(self.active) if self.active != exclude else None
        if p is not None and p.count() > (1 if self.active == exclude else 0):
            return p
        for q in self.root.findChildren(_Pane):
            if any(k != exclude for k in q.keys()):
                return q
        return None

    def _add_tab(self, pane: _Pane, key: str, index: int = -1) -> None:
        w = self.pages[key]
        if index < 0:
            pane.addTab(w, self.titles[key])
        else:
            pane.insertTab(index, w, self.titles[key])
        pane.setCurrentWidget(w)
        w.show()

    def _detach(self, key: str) -> None:
        """Take a page out of its pane without deleting it."""
        w = self.pages[key]
        p = self._pane_of(key)
        if p is not None:
            p.removeTab(p.indexOf(w))
        w.hide()
        w.setParent(self._park)

    def _take(self, key: str) -> None:
        """Remove a page from the layout and tidy up what it leaves behind."""
        p = self._pane_of(key)
        self._detach(key)
        if p is not None and p.count() == 0 and not self._rebuilding:
            self._drop_pane(p)

    def _drop_pane(self, p: _Pane) -> None:
        win = p.window()
        if isinstance(win, _FloatWindow):
            self._forget_float(win)
            win.close()
            win.deleteLater()
            return
        parent = p.parentWidget()
        p.setParent(None)
        p.deleteLater()
        self._collapse(parent)

    def _collapse(self, s) -> None:
        """A splitter left with one child is replaced by that child."""
        while isinstance(s, QSplitter) and s is not self.root and s.count() <= 1:
            parent = s.parentWidget()
            # (insertWidget, not replaceWidget: PySide does not hand the new
            # widget's ownership to the splitter on replaceWidget)
            if s.count() == 1 and isinstance(parent, QSplitter):
                sizes = parent.sizes()
                parent.insertWidget(parent.indexOf(s), s.widget(0))
                parent.setSizes(sizes)
            s.setParent(None)
            s.deleteLater()
            s = parent
        if s is self.root and self.root.count() == 1 and isinstance(self.root.widget(0), QSplitter):
            inner = self.root.widget(0)        # keep the root shallow
            self.root.setOrientation(inner.orientation())
            kids = [inner.widget(i) for i in range(inner.count())]
            sizes = inner.sizes()
            for k in kids:
                self.root.addWidget(k)
            inner.setParent(None); inner.deleteLater()
            self.root.setSizes(sizes)

    def _split(self, anchor: _Pane, key: str, orient) -> None:
        new = _Pane(self)
        self._add_tab(new, key)
        parent = anchor.parentWidget()
        if not isinstance(parent, QSplitter):           # anchor is in a float window
            parent = self.root
            parent.addWidget(new)
            return
        i = parent.indexOf(anchor)
        if parent.orientation() == orient or parent.count() == 1:
            parent.setOrientation(orient)
            sizes = parent.sizes()
            half = sizes[i] // 2 if sizes else 500
            parent.insertWidget(i + 1, new)
            if sizes:
                sizes[i] = sizes[i] - half
                sizes.insert(i + 1, half)
                parent.setSizes(sizes)
        else:
            sizes = parent.sizes()
            s = self._splitter(orient)
            parent.insertWidget(i, s)      # takes ownership; then move the anchor in
            s.addWidget(anchor)
            s.addWidget(new)
            s.setSizes([500, 500])
            parent.setSizes(sizes)

    def _float(self, keys: List[str], ref_geo, current: Optional[str] = None) -> _FloatWindow:
        fw = _FloatWindow(self)
        for k in keys:
            self._add_tab(fw.pane, k)
        if current in keys:
            fw.pane.setCurrentWidget(self.pages[current])
        if ref_geo is not None:
            fw.resize(max(640, ref_geo.width() // 2), max(420, ref_geo.height() // 2))
            fw.move(ref_geo.x() + ref_geo.width() // 4, ref_geo.y() + ref_geo.height() // 4)
        self.floats.append(fw)
        fw._retitle()
        fw.show()
        return fw

    def _forget_float(self, fw: _FloatWindow) -> None:
        if fw in self.floats:
            self.floats.remove(fw)
        self._schedule_emit()

    def _dock_back(self, key: str) -> None:
        self._take(key)
        anchor = self._anchor(exclude=key)
        self._add_tab(anchor if anchor is not None else self._root_pane(), key)
        self._set_active(key)
        self._update_tabbars()
        self._schedule_emit()

    def _update_tabbars(self) -> None:
        # one page alone in the window looks like before: no tab bar
        main = self.root.findChildren(_Pane)
        alone = len(main) == 1 and main[0].count() == 1
        for p in main:
            p.tabBar().setVisible(not alone)
        for f in self.floats:
            f.pane.tabBar().setVisible(True)

    # ---- tracking ----

    def _set_active(self, key: str) -> None:
        if key and key != self.active and key in self.pages:
            self.active = key
            self.active_changed.emit(key)

    def _pick_active(self) -> None:
        vis = [k for k in self.open_keys_ordered() if self.pages[k].isVisible()] or self.open_keys_ordered()
        if vis:
            self._set_active(vis[0])

    def _key_of(self, w) -> Optional[str]:
        while isinstance(w, QWidget):
            k = w.property("page_key")
            if k:
                return k
            if isinstance(w, _Pane):            # the tab bar of a pane
                return w.current_key()
            w = w.parentWidget()
        return None

    def eventFilter(self, obj: QObject, ev: QEvent) -> bool:
        t = ev.type()
        if t == QEvent.MouseButtonPress and isinstance(obj, QWidget):
            k = self._key_of(obj)
            if k:
                self._set_active(k)
        elif t in (QEvent.Show, QEvent.Hide) and isinstance(obj, QWidget) and obj.property("page_key"):
            # Qt shows widgets added to a visible window a little later:
            # report visibility when it really changes
            self._schedule_emit()
        return False

    def _on_focus(self, _old, now) -> None:
        k = self._key_of(now)
        if k:
            self._set_active(k)

    def _schedule_emit(self) -> None:
        # page visibility flips come in bursts (split, rebuild): report once
        if not self._emit_pending:
            self._emit_pending = True
            QTimer.singleShot(0, self._emit_visible)

    def _emit_visible(self) -> None:
        self._emit_pending = False
        self.visible_changed.emit()
