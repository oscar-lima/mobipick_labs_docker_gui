import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtCore import QRect, Qt
from PyQt5.QtWidgets import (
    QApplication,
    QPushButton,
    QSizePolicy,
    QWidget,
)

from mobipick_gui.flow_layout import FlowLayout


def _host(button_labels, width=400):
    app = QApplication.instance() or QApplication([])
    host = QWidget()
    layout = FlowLayout(host, horizontal_spacing=6, vertical_spacing=6)
    buttons = []
    for label in button_labels:
        button = QPushButton(label)
        button.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        layout.addWidget(button)
        buttons.append(button)
    host.resize(width, 400)
    app.processEvents()
    return app, host, layout, buttons


def test_minimum_width_is_the_widest_single_item():
    labels = ['Start Simulation', 'RViz', 'RQt', 'Tables Demo', 'Terminal']
    app, host, layout, buttons = _host(labels)

    widest = max(button.minimumSizeHint().width() for button in buttons)
    natural = layout.sizeHint().width()

    # A QHBoxLayout would demand the full single-line width; the flow layout
    # only needs room for its widest child, so the window can be shrunk.
    assert layout.minimumSize().width() == widest
    assert natural > widest

    host.deleteLater()
    app.processEvents()


def test_items_wrap_onto_additional_lines_when_the_row_is_narrow():
    labels = ['Start Simulation', 'Start RViz', 'Start RQt', 'Tables Demo']
    app, host, layout, buttons = _host(labels)

    wide = layout.sizeHint().width()
    layout.setGeometry(QRect(0, 0, wide, 400))
    tops = {button.geometry().top() for button in buttons}
    assert len(tops) == 1
    assert layout.heightForWidth(wide) < layout.heightForWidth(wide // 3)

    layout.setGeometry(QRect(0, 0, wide // 3, 400))
    rows = sorted({button.geometry().top() for button in buttons})
    assert len(rows) > 1
    for button in buttons:
        assert button.geometry().right() <= wide // 3

    host.deleteLater()
    app.processEvents()


def test_expanding_items_share_the_leftover_space_of_their_line():
    app, host, layout, buttons = _host(['One', 'Two'])

    layout.setGeometry(QRect(0, 0, 400, 400))

    first, second = (button.geometry() for button in buttons)
    assert first.left() == 0
    assert second.right() == 399
    assert abs(first.width() - second.width()) <= 1

    host.deleteLater()
    app.processEvents()


def test_insert_widget_keeps_the_requested_order():
    app, host, layout, buttons = _host(['First', 'Last'])

    inserted = QPushButton('Middle')
    layout.insertWidget(1, inserted)
    app.processEvents()

    assert layout.indexOf(inserted) == 1
    assert layout.count() == 3
    assert inserted.parent() is host

    layout.removeWidget(inserted)
    assert layout.count() == 2
    assert layout.indexOf(buttons[1]) == 1

    host.deleteLater()
    app.processEvents()


def test_layout_expands_horizontally_and_reports_height_for_width():
    app, host, layout, _buttons = _host(['Alpha', 'Beta', 'Gamma'])

    assert layout.hasHeightForWidth()
    assert layout.expandingDirections() & Qt.Horizontal

    host.deleteLater()
    app.processEvents()
