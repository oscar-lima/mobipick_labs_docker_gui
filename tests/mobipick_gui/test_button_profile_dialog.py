import os
from pathlib import Path

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication

from mobipick_gui.main_window import ButtonProfileDialog


def test_button_profile_dialog_prioritizes_command_columns(tmp_path):
    app = QApplication.instance() or QApplication([])
    dialog = ButtonProfileDialog(
        [
            {
                'key': 'demo_tool',
                'label': 'Demo Tool',
                'kind': 'command',
                'command': 'rosrun demo_package very_long_command_name --flag value',
                'tooltip': (
                    'This tooltip is intentionally much longer than the '
                    'other fields and should not dominate the visible width.'
                ),
            },
        ],
        Path(tmp_path / 'source.yaml'),
        Path(tmp_path / 'target.yaml'),
    )
    dialog.table.resize(720, 240)
    dialog.table.apply_column_widths()

    widths = {
        field: dialog.table.columnWidth(index)
        for index, (field, _label) in enumerate(dialog.COLUMNS)
    }

    assert widths['command'] > widths['tooltip']
    assert widths['label'] > widths['tooltip']
    assert widths['key'] >= 96

    dialog.deleteLater()
    app.processEvents()
