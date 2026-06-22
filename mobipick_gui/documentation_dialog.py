"""User documentation dialog with keyword search."""
from __future__ import annotations

from pathlib import Path

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QTextCharFormat, QTextCursor, QTextDocument
from PyQt5.QtWidgets import (
    QDialog,
    QDialogButtonBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QTextBrowser,
    QTextEdit,
    QVBoxLayout,
    QWidget,
)

from .external_links import open_external_url


class DocumentationDialog(QDialog):
    """Render user documentation and search within it."""

    def __init__(self, markdown_path: Path, parent: QWidget | None = None):
        super().__init__(parent)
        self.setWindowTitle('Mobipick GUI Documentation')
        self.setWindowModality(Qt.NonModal)
        self.resize(980, 720)

        self._markdown_path = Path(markdown_path)
        self._last_pattern = ''

        root = QVBoxLayout(self)

        search_row = QHBoxLayout()
        search_row.addWidget(QLabel('Search:'))
        self.search_input = QLineEdit()
        self.search_input.setPlaceholderText('Keyword')
        self.search_input.returnPressed.connect(self.find_next)
        search_row.addWidget(self.search_input, 1)

        self.find_button = QPushButton('Find')
        self.find_button.clicked.connect(self.find_next)
        search_row.addWidget(self.find_button)

        self.previous_button = QPushButton('Previous')
        self.previous_button.clicked.connect(self.find_previous)
        search_row.addWidget(self.previous_button)

        self.match_label = QLabel('')
        search_row.addWidget(self.match_label)
        root.addLayout(search_row)

        self.viewer = QTextBrowser()
        self.viewer.setOpenLinks(False)
        self.viewer.setOpenExternalLinks(False)
        self.viewer.anchorClicked.connect(self._open_documentation_link)
        root.addWidget(self.viewer, 1)

        buttons = QDialogButtonBox(QDialogButtonBox.Close)
        buttons.rejected.connect(self.reject)
        root.addWidget(buttons)

        self._load_documentation()

    def _load_documentation(self) -> None:
        try:
            markdown = self._markdown_path.read_text(encoding='utf-8')
        except OSError as exc:
            self.viewer.setPlainText(
                'User documentation could not be loaded.\n\n'
                f'{self._markdown_path}\n\n{exc}'
            )
            return

        if hasattr(self.viewer, 'setMarkdown'):
            self.viewer.setMarkdown(markdown)
        else:  # pragma: no cover - Qt versions older than the package target
            self.viewer.setPlainText(markdown)
        cursor = self.viewer.textCursor()
        cursor.movePosition(QTextCursor.Start)
        self.viewer.setTextCursor(cursor)

    def find_next(self) -> None:
        self._find(QTextDocument.FindFlags())

    def find_previous(self) -> None:
        self._find(QTextDocument.FindBackward)

    def _find(self, flags=QTextDocument.FindFlags()) -> None:
        pattern = self.search_input.text().strip()
        if not pattern:
            self._clear_highlights()
            return

        match_count = self._highlight_matches(pattern)
        if pattern != self._last_pattern:
            cursor = self.viewer.textCursor()
            cursor.movePosition(
                QTextCursor.End
                if flags & QTextDocument.FindBackward
                else QTextCursor.Start
            )
            self.viewer.setTextCursor(cursor)
            self._last_pattern = pattern

        found = self.viewer.find(pattern, flags)
        if not found:
            cursor = self.viewer.textCursor()
            cursor.movePosition(
                QTextCursor.End
                if flags & QTextDocument.FindBackward
                else QTextCursor.Start
            )
            self.viewer.setTextCursor(cursor)
            found = self.viewer.find(pattern, flags)

        if found:
            self.viewer.ensureCursorVisible()
            return

        self.match_label.setText('0 matches')
        QMessageBox.information(self, 'Search', f'No match for "{pattern}"')

    def _highlight_matches(self, pattern: str) -> int:
        selections: list[QTextEdit.ExtraSelection] = []
        document = self.viewer.document()
        cursor = QTextCursor(document)
        highlight = QTextCharFormat()
        highlight.setBackground(QColor('#fff59d'))
        highlight.setForeground(QColor('#111111'))

        while True:
            cursor = document.find(pattern, cursor)
            if cursor.isNull():
                break
            selection = QTextEdit.ExtraSelection()
            selection.cursor = cursor
            selection.format = highlight
            selections.append(selection)

        self.viewer.setExtraSelections(selections)
        count = len(selections)
        self.match_label.setText(f'{count} match' + ('' if count == 1 else 'es'))
        return count

    def _clear_highlights(self) -> None:
        self.viewer.setExtraSelections([])
        self.match_label.clear()
        self._last_pattern = ''

    def _open_documentation_link(self, url) -> None:
        if url.isRelative() and not url.scheme():
            self.viewer.setSource(url)
            return
        if not open_external_url(url):
            QMessageBox.warning(
                self,
                'Documentation',
                'Unable to open the selected link.',
            )


__all__ = ['DocumentationDialog']
