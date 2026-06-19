from pathlib import Path

import mobipick_gui.config as config_module
from mobipick_gui.config import (
    CONFIG,
    CONFIG_FILE,
    default_user_config_dir,
    default_user_data_dir,
    load_button_layout,
    load_docker_cp_config,
    load_launch_sequence_plan,
    save_button_layout,
    save_docker_cp_config,
    save_launch_sequence_plan,
    writable_button_config_path,
    writable_launch_sequence_path,
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


def test_bundled_gui_settings_exclude_private_runtime_state():
    text = CONFIG_FILE.read_text(encoding='utf-8')

    assert 'completed:' not in text
    assert 'oscar_user_from_1.2' not in text
    assert 'rae_ws_from_oscar_user' not in text
    assert 'gpt_ws_from_oscar_user' not in text


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

    assert [entry['key'] for entry in layout] == ['sim', 'custom', 'rviz']
    assert plan['source'] == str(launches)
    assert plan['timeline'] == [{'button': 'custom', 'at_seconds': 1.0}]
    assert plan['recording_start_delay_seconds'] == 0.0


def test_button_layout_keeps_sim_and_rviz_required(tmp_path):
    buttons = tmp_path / 'buttons.yaml'
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

    layout = load_button_layout(Path(buttons))

    assert [entry['key'] for entry in layout] == ['sim', 'custom', 'rviz']


def test_save_button_layout_persists_required_buttons(tmp_path):
    target = tmp_path / 'buttons.yaml'

    save_button_layout(
        target,
        [
            {
                'key': 'custom',
                'label': 'Custom',
                'kind': 'command',
                'command': 'echo custom',
            }
        ],
    )

    text = target.read_text(encoding='utf-8')
    assert 'key: sim' in text
    assert 'key: custom' in text
    assert 'key: rviz' in text


def test_writable_button_config_path_avoids_packaged_resources(
    monkeypatch,
    tmp_path,
):
    profile_dir = tmp_path / 'button_profiles'
    monkeypatch.setattr(config_module, 'BUTTON_PROFILE_DIR', profile_dir)
    source = config_module.PROJECT_ROOT / 'config' / 'buttons.yaml'

    assert writable_button_config_path(source) == profile_dir / 'buttons.yaml'


def test_launch_sequence_persists_recording_start_delay(tmp_path):
    launches = tmp_path / 'launches.yaml'

    save_launch_sequence_plan(
        launches,
        [{'button': 'custom', 'at_seconds': 1.5}],
        ['custom'],
        {'label': 'Auto Launch'},
        3.25,
    )

    text = launches.read_text(encoding='utf-8')
    assert 'recording:' in text
    assert 'start_delay_seconds: 3.25' in text

    plan = load_launch_sequence_plan(None, launches)
    assert plan['recording_start_delay_seconds'] == 3.25


def test_relative_launch_config_can_load_user_config_fallback(monkeypatch, tmp_path):
    launch_dir = tmp_path / 'launch_sequences'
    monkeypatch.setattr(config_module, 'LAUNCH_SEQUENCE_DIR', launch_dir)
    buttons = tmp_path / 'buttons.yaml'
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
    launches = launch_dir / 'custom_auto.yaml'
    launches.parent.mkdir(parents=True)
    launches.write_text(
        '''
timeline:
  - at_seconds: 2
    button: custom
''',
        encoding='utf-8',
    )

    plan = load_launch_sequence_plan(buttons, 'custom_auto.yaml')

    assert plan['source'] == str(launches)
    assert plan['timeline'] == [{'button': 'custom', 'at_seconds': 2.0}]


def test_writable_launch_sequence_path_avoids_packaged_resources(monkeypatch, tmp_path):
    launch_dir = tmp_path / 'launch_sequences'
    monkeypatch.setattr(config_module, 'LAUNCH_SEQUENCE_DIR', launch_dir)
    source = config_module.PROJECT_ROOT / 'config' / 'custom_auto.yaml'

    assert writable_launch_sequence_path(source) == launch_dir / 'custom_auto.yaml'


def test_docker_cp_user_config_overrides_bundled_config(monkeypatch, tmp_path):
    bundled = tmp_path / 'bundled.yaml'
    user = tmp_path / 'user.yaml'
    bundled.write_text(
        '''
default:
  container_to_host:
    - container: /container/default.rviz
      host: ~/Downloads/default.rviz
image:tag:
  host_to_container:
    - host: ~/Downloads/bundled.rviz
      container: /container/bundled.rviz
''',
        encoding='utf-8',
    )
    user.write_text(
        '''
image:tag:
  host_to_container:
    - host: ~/Downloads/user.rviz
      container: /container/user.rviz
''',
        encoding='utf-8',
    )
    monkeypatch.setattr(config_module, 'DOCKER_CP_CONFIG_FILE', bundled)
    monkeypatch.setattr(config_module, 'USER_DOCKER_CP_CONFIG_FILE', user)

    config = load_docker_cp_config()

    assert config['default']['container_to_host'] == [
        {
            'container': '/container/default.rviz',
            'host': '~/Downloads/default.rviz',
        }
    ]
    assert config['image:tag']['host_to_container'] == [
        {
            'container': '/container/user.rviz',
            'host': '~/Downloads/user.rviz',
        }
    ]


def test_save_docker_cp_config_writes_user_file(monkeypatch, tmp_path):
    target = tmp_path / 'config' / 'docker_cp_image_tag.yaml'
    monkeypatch.setattr(config_module, 'USER_DOCKER_CP_CONFIG_FILE', target)

    saved = save_docker_cp_config(
        {
            ' default ': {
                'host_to_container': [
                    {
                        'host': ' ~/Downloads/source.rviz ',
                        'container': ' /container/source.rviz ',
                    },
                    {
                        'host': '',
                        'container': '/ignored',
                    },
                ],
                'container_to_host': [],
            },
            'empty': {
                'host_to_container': [],
                'container_to_host': [],
            },
        }
    )

    assert saved == target
    text = target.read_text(encoding='utf-8')
    assert 'default:' in text
    assert 'host: ~/Downloads/source.rviz' in text
    assert 'container: /container/source.rviz' in text
    assert 'empty:' in text


def test_empty_docker_cp_user_profile_disables_bundled_default(
    monkeypatch,
    tmp_path,
):
    bundled = tmp_path / 'bundled.yaml'
    user = tmp_path / 'user.yaml'
    bundled.write_text(
        '''
default:
  host_to_container:
    - host: ~/Downloads/source.rviz
      container: /container/source.rviz
''',
        encoding='utf-8',
    )
    user.write_text(
        '''
default:
  host_to_container: []
  container_to_host: []
''',
        encoding='utf-8',
    )
    monkeypatch.setattr(config_module, 'DOCKER_CP_CONFIG_FILE', bundled)
    monkeypatch.setattr(config_module, 'USER_DOCKER_CP_CONFIG_FILE', user)

    config = load_docker_cp_config()

    assert config['default'] == {
        'host_to_container': [],
        'container_to_host': [],
    }
