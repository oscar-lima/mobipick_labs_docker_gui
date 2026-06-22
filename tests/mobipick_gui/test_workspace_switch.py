import copy
import os

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5.QtWidgets import QApplication, QMessageBox

from mobipick_gui.config import CONFIG
from mobipick_gui.main_window import MainWindow
from mobipick_gui.workspaces import RosWorkspace, WorkspaceRegistry


def test_workspace_switch_requires_confirmation_and_rebuilds_tabs(
    tmp_path,
    monkeypatch,
):
    gpt_buttons = tmp_path / 'gpt_buttons.yaml'
    gpt_buttons.write_text(
        '''
buttons:
  - key: gpt-action
    label: GPT Action
    kind: command
    command: echo gpt
''',
        encoding='utf-8',
    )
    rae_buttons = tmp_path / 'rae_buttons.yaml'
    rae_buttons.write_text(
        '''
buttons:
  - key: rae-action
    label: RAE Action
    kind: command
    command: echo rae
''',
        encoding='utf-8',
    )

    registry_path = tmp_path / 'workspaces.yaml'
    registry = WorkspaceRegistry(registry_path)
    registry.upsert(
        RosWorkspace(
            name='gpt_ws',
            path=str(tmp_path / 'gpt_ws'),
            button_config=str(gpt_buttons),
            image='example/mobipick:gpt',
        )
    )
    registry.upsert(
        RosWorkspace(
            name='rae_upom_mobipick_ws',
            path=str(tmp_path / 'rae_upom_mobipick_ws'),
            button_config=str(rae_buttons),
            image='example/mobipick:rae',
        )
    )
    registry.active = 'gpt_ws'
    registry.save()

    images = [
        {'ref': 'ozkrelo/mobipick_labs:noetic'},
        {'ref': 'example/mobipick:gpt'},
        {'ref': 'example/mobipick:rae'},
    ]
    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(registry_path))
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: (images, None),
    )
    monkeypatch.setattr(
        MainWindow,
        'update_sim_status_from_poll',
        lambda self, force=False: None,
    )

    app = QApplication.instance() or QApplication([])
    window = MainWindow(verbosity=1)
    window.poll_timer.stop()
    window._sigint_timer.stop()
    stale_output = window._ensure_tab(
        'custom-stale',
        'Custom Stale',
        closable=True,
    ).output

    monkeypatch.setattr(
        QMessageBox,
        'question',
        lambda *args, **kwargs: QMessageBox.Cancel,
    )
    assert not window._activate_workspace('rae_upom_mobipick_ws')
    assert window._workspace_registry.active == 'gpt_ws'
    assert 'custom-stale' in window.tasks

    monkeypatch.setattr(
        QMessageBox,
        'question',
        lambda *args, **kwargs: QMessageBox.Yes,
    )
    assert window._activate_workspace('rae_upom_mobipick_ws')
    assert window._workspace_registry.active == 'rae_upom_mobipick_ws'
    assert window._selected_image == 'example/mobipick:rae'
    assert window._config_button_order == ['sim', 'rae-action', 'rviz']
    assert 'custom-stale' not in window.tasks
    assert window.tabs.indexOf(stale_output) == -1
    env_args = window._compose_env_args()
    assert not any(
        'MOBIPICK_WORKSPACE_MOUNT_SOURCE=' in argument
        for argument in env_args
    )
    assert not any(
        'MOBIPICK_WORKSPACE_COMPAT_ROOTS=' in argument
        for argument in env_args
    )
    assert any(
        'MOBIPICK_WORKSPACE_MOUNT_TARGET=' in argument
        for argument in env_args
    )

    window.deleteLater()
    app.processEvents()


def test_workspace_switch_selects_first_matching_image(tmp_path, monkeypatch):
    images_cfg = copy.deepcopy(CONFIG['images'])
    images_cfg['default'] = 'example/mobipick:default'
    images_cfg['profiles'] = [
        {
            'ref': 'example/mobipick:default',
            'user': 'root',
            'supports_host_workspaces': False,
            'compatible_workspaces': ['Docker image default'],
        },
        {
            'ref': 'example/mobipick:gpt',
            'user': 'host',
            'supports_host_workspaces': True,
            'compatible_workspaces': ['gpt_ws'],
        },
        {
            'ref': 'example/mobipick:rae-first',
            'user': 'host',
            'supports_host_workspaces': True,
            'compatible_workspaces': ['rae_upom_mobipick_ws'],
        },
        {
            'ref': 'example/mobipick:rae-second',
            'user': 'host',
            'supports_host_workspaces': True,
            'compatible_workspaces': ['rae_upom_mobipick_ws'],
        },
    ]
    monkeypatch.setitem(CONFIG, 'images', images_cfg)

    registry_path = tmp_path / 'workspaces.yaml'
    registry = WorkspaceRegistry(registry_path)
    registry.upsert(
        RosWorkspace(
            name='gpt_ws',
            path=str(tmp_path / 'gpt_ws'),
        )
    )
    registry.upsert(
        RosWorkspace(
            name='rae_upom_mobipick_ws',
            path=str(tmp_path / 'rae_upom_mobipick_ws'),
            image='example/mobipick:legacy-registered',
        )
    )
    registry.active = 'gpt_ws'
    registry.save()

    images = [
        {'ref': 'example/mobipick:default'},
        {'ref': 'example/mobipick:gpt'},
        {'ref': 'example/mobipick:rae-first'},
        {'ref': 'example/mobipick:rae-second'},
        {'ref': 'example/mobipick:legacy-registered'},
    ]
    monkeypatch.setenv('MOBIPICK_WORKSPACE_CONFIG', str(registry_path))
    monkeypatch.setattr(
        MainWindow,
        '_discover_filtered_image_records',
        lambda self: (images, None),
    )
    monkeypatch.setattr(
        MainWindow,
        'update_sim_status_from_poll',
        lambda self, force=False: None,
    )
    monkeypatch.setattr(
        QMessageBox,
        'question',
        lambda *args, **kwargs: QMessageBox.Yes,
    )

    app = QApplication.instance() or QApplication([])
    window = MainWindow(verbosity=1)
    window.poll_timer.stop()
    window._sigint_timer.stop()

    assert window._selected_image == 'example/mobipick:gpt'

    assert window._activate_workspace('rae_upom_mobipick_ws')

    assert window._workspace_registry.active == 'rae_upom_mobipick_ws'
    assert window._selected_image == 'example/mobipick:rae-first'

    window.deleteLater()
    app.processEvents()
