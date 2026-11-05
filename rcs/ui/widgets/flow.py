"""Layout that wraps its widgets onto the next line when it runs out of width,
so a row of controls does not force a split pane to stay wide."""
from __future__ import annotations

from PySide6.QtCore import QPoint, QRect, QSize, Qt
from PySide6.QtWidgets import QLayout


class FlowLayout(QLayout):
    def __init__(self, parent=None, spacing: int = 6):
        super().__init__(parent)
        self._items: list = []
        self.setContentsMargins(0, 0, 0, 0)
        self.setSpacing(spacing)

    def addItem(self, item) -> None:
        self._items.append(item)

    def count(self) -> int:
        return len(self._items)

    def itemAt(self, i: int):
        return self._items[i] if 0 <= i < len(self._items) else None

    def takeAt(self, i: int):
        return self._items.pop(i) if 0 <= i < len(self._items) else None

    def expandingDirections(self):
        return Qt.Orientation(0)

    def hasHeightForWidth(self) -> bool:
        return True

    def heightForWidth(self, width: int) -> int:
        return self._arrange(QRect(0, 0, width, 0), apply=False)

    def setGeometry(self, rect: QRect) -> None:
        super().setGeometry(rect)
        self._arrange(rect, apply=True)

    def sizeHint(self) -> QSize:
        return self.minimumSize()

    def minimumSize(self) -> QSize:
        s = QSize()
        for it in self._items:
            s = s.expandedTo(it.minimumSize())
        return s

    def _arrange(self, rect: QRect, apply: bool) -> int:
        """Lay out line by line, each item centered in its line's height."""
        gap = self.spacing()
        lines, line, x = [], [], rect.x()
        for it in self._items:
            w = it.sizeHint().width()
            if line and x + w > rect.right():
                lines.append(line)
                line, x = [], rect.x()
            line.append(it)
            x += w + gap
        if line:
            lines.append(line)
        y = rect.y()
        for line in lines:
            h = max(it.sizeHint().height() for it in line)
            x = rect.x()
            for it in line:
                hint = it.sizeHint()
                if apply:
                    it.setGeometry(QRect(QPoint(x, y + (h - hint.height()) // 2), hint))
                x += hint.width() + gap
            y += h + gap
        return y - gap - rect.y() if lines else 0
