from pathlib import Path

from mobipick_gui.config import (
    CONFIG,
    default_user_config_dir,
    default_user_data_dir,
    load_button_layout,
    load_launch_sequence_plan,
)


def test_screen_recording_is_disabled_by_default():
    assert CONFIG['recording']['enabled_by_default'] is False


def test_user_state_uses_xdg_directories(monkeypatch, tmp_path):
    config_home = tmp_path / 'config'
    data_home = tmp_path / 'data'
    monkeypatch.setenv('XDG_CONFIG_HOME', str(config_home))
    monkeypatch.setenv('XDG_DATA_HOME', str(data_home))

    assert default_user_config_dir() == (
        config_home / 'mobipick-labs-docker-gui'
    )
    assert default_user_data_dir() == (
        data_home / 'mobipick-labs-docker-gui'
    )


def test_explicit_workspace_profiles_override_global_config(tmp_path):
    buttons = tmp_path / 'buttons.yaml'
    launches = tmp_path / 'launches.yaml'
    buttons.write_text(
        '''
buttons:
  - key: custom
    label: Custom
    kind: command
    command: echo custom
''',
        encoding='utf-8',
    )
    launches.write_text(
        '''
timeline:
  - at_seconds: 1
    button: custom
''',
        encoding='utf-8',
    )

    layout = load_button_layout(Path(buttons))
    plan = load_launch_sequence_plan(buttons, launches)

    assert [entry['key'] for entry in layout] == ['custom']
    assert plan['source'] == str(launches)
    assert plan['timeline'] == [{'button': 'custom', 'at_seconds': 1.0}]
