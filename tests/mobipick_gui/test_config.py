from pathlib import Path

import yaml

import mobipick_gui.config as config_module
from mobipick_gui.config import (
    CONFIG,
    CONFIG_DEFAULTS,
    CONFIG_FILE,
    DEFAULT_BUTTON_COMMANDS,
    default_user_config_dir,
    default_user_data_dir,
    load_button_layout,
    load_docker_cp_config,
    load_launch_sequence_plan,
    load_user_config_overrides,
    save_button_layout,
    save_docker_cp_config,
    save_launch_sequence_plan,
    save_user_config_update,
    user_configuration_paths,
    user_state_reset_command,
    user_state_reset_paths,
    writable_button_config_path,
    writable_workspace_button_config_path,
    writable_workspace_docker_cp_config_path,
    writable_launch_sequence_path,
)


def test_screen_recording_is_disabled_by_default():
    assert CONFIG['recording']['enabled_by_default'] is False


def test_image_setup_defaults_target_noetic_v2():
    assert CONFIG_DEFAULTS['images']['default'] == (
        'ozkrelo/x_mobipick_labs:noetic-v2.0'
    )
    assert CONFIG_DEFAULTS['images']['discovery_filters'] == ['mobipick_labs']
    assert CONFIG_DEFAULTS['setup_wizard']['development_base_image'] == (
        'ozkrelo/x_mobipick_labs:noetic-v2.0'
    )
    assert (
        CONFIG_DEFAULTS['setup_wizard']['development_image_tag_template']
        == '{user}_user_from_2.0'
    )


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


def test_user_configuration_paths_include_managed_state(tmp_path):
    registry = tmp_path / 'workspaces.yaml'
    layout = tmp_path / 'layouts' / '{workspace}.yaml'

    labels = {
        label: path
        for label, path in user_configuration_paths(
            workspace_registry_path=registry,
            window_layout_template=layout,
        )
    }

    assert labels['Workspace registry'] == registry
    assert labels['Window layouts'] == layout
    assert labels['GUI settings'].name == 'gui_settings.yaml'
    assert labels['Docker cp paths'].name == 'docker_cp_image_tag.yaml'
    assert labels['Workspace Docker cp profiles'].name == 'docker_cp_profiles'
    assert labels['Button profiles'].name == 'button_profiles'
    assert labels['Auto-launch profiles'].name == 'launch_sequences'


def test_user_state_reset_paths_are_top_level_xdg_roots(monkeypatch, tmp_path):
    config_home = tmp_path / 'config space'
    data_home = tmp_path / 'data'
    monkeypatch.setenv('XDG_CONFIG_HOME', str(config_home))
    monkeypatch.setenv('XDG_DATA_HOME', str(data_home))

    assert user_state_reset_paths() == [
        config_home / 'mobipick-labs-docker-gui',
        data_home / 'mobipick-labs-docker-gui',
    ]


def test_user_state_reset_command_names_targets_and_requires_confirmation(
    monkeypatch,
    tmp_path,
):
    config_home = tmp_path / 'config space'
    data_home = tmp_path / 'data'
    monkeypatch.setenv('XDG_CONFIG_HOME', str(config_home))
    monkeypatch.setenv('XDG_DATA_HOME', str(data_home))

    command = user_state_reset_command()

    assert 'DELETE_MOBIPICK_GUI_CONFIG' in command
    assert 'rm -rf --' in command
    assert str(config_home / 'mobipick-labs-docker-gui') in command
    assert str(data_home / 'mobipick-labs-docker-gui') in command
    assert f"'{config_home / 'mobipick-labs-docker-gui'}'" in command


def test_bundled_gui_settings_exclude_private_runtime_state():
    text = CONFIG_FILE.read_text(encoding='utf-8')

    assert 'completed:' not in text
    assert 'host_user_from_1.2' not in text
    assert 'rae_ws_from_host_user' not in text
    assert 'gpt_ws_from_host_user' not in text


def test_legacy_host_user_values_are_migrated_from_user_config(
    monkeypatch,
    tmp_path,
):
    legacy_user = 'osc' + 'ar'
    user_config = tmp_path / 'gui_settings.yaml'
    user_config.write_text(
        f'''
images:
  default: ozkrelo/x_mobipick_labs:gpt_ws_from_{legacy_user}_user
  profiles:
    - ref: ozkrelo/x_mobipick_labs:{legacy_user}_user_from_1.2
      user: host
      compatible_workspaces: [clean_mobipick_labs_ws]
setup_wizard:
  host_user: {legacy_user}
custom:
  path: /home/{legacy_user}/ros_ws/gpt_ws
''',
        encoding='utf-8',
    )
    monkeypatch.setattr(config_module, 'USER_CONFIG_FILE', user_config)
    monkeypatch.setattr(config_module, 'HOST_USER', 'tester')

    config = load_user_config_overrides()

    assert config['images']['default'] == (
        'ozkrelo/x_mobipick_labs:gpt_ws_from_tester_user'
    )
    assert config['images']['profiles'][0]['ref'] == (
        'ozkrelo/x_mobipick_labs:tester_user_from_1.2'
    )
    assert config['setup_wizard']['host_user'] == 'tester'
    assert config['custom']['path'] == '/home/tester/ros_ws/gpt_ws'


def test_saved_user_config_update_does_not_persist_legacy_host_user(
    monkeypatch,
    tmp_path,
):
    legacy_user = 'osc' + 'ar'
    user_config = tmp_path / 'gui_settings.yaml'
    monkeypatch.setattr(config_module, 'USER_CONFIG_FILE', user_config)
    monkeypatch.setattr(config_module, 'HOST_USER', 'tester')
    monkeypatch.setattr(config_module, 'CONFIG', {})

    save_user_config_update(
        {
            'images': {
                'default': (
                    f'ozkrelo/x_mobipick_labs:gpt_ws_from_{legacy_user}_user'
                ),
            }
        }
    )

    text = user_config.read_text(encoding='utf-8')
    assert legacy_user not in text
    assert 'gpt_ws_from_tester_user' in text


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
    assert layout[0]['command'] == DEFAULT_BUTTON_COMMANDS['sim']
    assert layout[2]['command'] == DEFAULT_BUTTON_COMMANDS['rviz']
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
    assert layout[0]['command'] == DEFAULT_BUTTON_COMMANDS['sim']
    assert layout[2]['command'] == DEFAULT_BUTTON_COMMANDS['rviz']


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
    data = yaml.safe_load(text)
    commands = {
        entry['key']: entry.get('command')
        for entry in data['buttons']
    }
    assert 'key: sim' in text
    assert 'key: custom' in text
    assert 'key: rviz' in text
    assert 'reuse_tab:' not in text
    assert 'requires_roscore:' not in text
    assert 'pass_ros_master_uri:' not in text
    assert commands['sim'] == DEFAULT_BUTTON_COMMANDS['sim']
    assert commands['rviz'] == DEFAULT_BUTTON_COMMANDS['rviz']


def test_button_layout_save_load_round_trip_single_file(tmp_path):
    target = tmp_path / 'exported_buttons.yaml'
    save_button_layout(
        target,
        [
            {
                'key': 'sim',
                'label': 'Sim',
                'kind': 'builtin',
                'action': 'sim',
                'command': 'roslaunch custom sim.launch',
                'tooltip': 'Custom sim',
            },
            {
                'key': 'custom_tool',
                'label': 'Custom Tool',
                'kind': 'command',
                'command': 'rosrun custom tool.py',
                'tooltip': 'Run custom tool',
                'service': 'mobipick',
            },
            {
                'key': 'rviz',
                'label': 'RViz',
                'kind': 'builtin',
                'action': 'rviz',
                'command': 'rviz -d /tmp/custom.rviz',
                'tooltip': 'Custom rviz',
            },
        ],
    )

    loaded = load_button_layout(target)
    by_key = {entry['key']: entry for entry in loaded}

    assert list(by_key) == ['sim', 'custom_tool', 'rviz']
    assert by_key['sim']['command'] == 'roslaunch custom sim.launch'
    assert by_key['custom_tool']['command'] == 'rosrun custom tool.py'
    assert by_key['custom_tool']['tooltip'] == 'Run custom tool'
    assert by_key['custom_tool']['service'] == 'mobipick'
    assert by_key['rviz']['command'] == 'rviz -d /tmp/custom.rviz'


def test_button_layout_round_trips_three_optional_generic_args(tmp_path):
    target = tmp_path / 'generic_args.yaml'
    save_button_layout(
        target,
        [
            {
                'key': 'sim',
                'label': 'Sim',
                'kind': 'builtin',
                'action': 'sim',
                'command': 'roslaunch demo sim.launch',
                'arg_1_name': 'robot',
                'arg_1_options': ['thor', 'panda'],
                'arg_1_applies': True,
                'arg_2_name': 'map_file',
                'arg_2_options': ['/tmp/a.yaml', '/tmp/b.yaml'],
                'arg_2_applies': False,
                'arg_3_name': 'mode',
                'arg_3_options': ['demo', 'live'],
                'arg_3_applies': True,
            },
            {
                'key': 'rviz',
                'label': 'RViz',
                'kind': 'builtin',
                'action': 'rviz',
                'command': 'rviz',
            },
        ],
    )

    loaded = {entry['key']: entry for entry in load_button_layout(target)}

    assert loaded['sim']['arg_1_name'] == 'robot'
    assert loaded['sim']['arg_1_options'] == ['thor', 'panda']
    assert loaded['sim']['arg_1_applies'] is True
    assert loaded['sim']['arg_2_name'] == 'map_file'
    assert loaded['sim']['arg_2_options'] == [
        '/tmp/a.yaml',
        '/tmp/b.yaml',
    ]
    assert loaded['sim']['arg_2_applies'] is False
    assert loaded['sim']['arg_3_name'] == 'mode'
    assert loaded['sim']['arg_3_options'] == ['demo', 'live']
    assert loaded['sim']['arg_3_applies'] is True


def test_writable_button_config_path_avoids_packaged_resources(
    monkeypatch,
    tmp_path,
):
    profile_dir = tmp_path / 'button_profiles'
    monkeypatch.setattr(config_module, 'BUTTON_PROFILE_DIR', profile_dir)
    source = config_module.PROJECT_ROOT / 'config' / 'buttons.yaml'

    assert writable_button_config_path(source) == profile_dir / 'buttons.yaml'


def test_writable_workspace_button_config_path_is_workspace_specific(
    monkeypatch,
    tmp_path,
):
    profile_dir = tmp_path / 'button_profiles'
    monkeypatch.setattr(config_module, 'BUTTON_PROFILE_DIR', profile_dir)
    source = config_module.PROJECT_ROOT / 'config' / 'button_commands_labs.yaml'

    assert writable_workspace_button_config_path(
        source,
        'gpt_ws',
    ) == profile_dir / 'gpt_ws_button_commands_labs.yaml'


def test_writable_workspace_button_config_path_keeps_existing_workspace_copy(
    monkeypatch,
    tmp_path,
):
    profile_dir = tmp_path / 'button_profiles'
    monkeypatch.setattr(config_module, 'BUTTON_PROFILE_DIR', profile_dir)
    source = profile_dir / 'gpt_ws_button_commands_labs.yaml'

    assert writable_workspace_button_config_path(source, 'gpt_ws') == source


def test_writable_workspace_docker_cp_config_path_is_workspace_specific(
    monkeypatch,
    tmp_path,
):
    profile_dir = tmp_path / 'docker_cp_profiles'
    monkeypatch.setattr(config_module, 'DOCKER_CP_PROFILE_DIR', profile_dir)

    assert writable_workspace_docker_cp_config_path('gpt ws') == (
        profile_dir / 'gpt_ws_docker_cp_image_tag.yaml'
    )


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


def test_legacy_launch_sequence_loads_in_legacy_mode(tmp_path):
    launches = tmp_path / 'legacy.yaml'
    launches.write_text(
        'timeline:\n  - button: sim\n    at_seconds: 2\n',
        encoding='utf-8',
    )

    plan = load_launch_sequence_plan(None, launches)

    assert plan['mode'] == 'legacy'
    assert plan['processes'] == []
    assert plan['timeline'] == [{'button': 'sim', 'at_seconds': 2.0}]


def test_advanced_launch_sequence_round_trip(tmp_path):
    launches = tmp_path / 'advanced.yaml'
    processes = [
        {
            'button': 'sim',
            'duration_seconds': 20.0,
            'depends_on': 'roscore',
            'dependency_type': 'soft',
            'ready_percentage': 30.0,
        }
    ]

    save_launch_sequence_plan(
        launches,
        [],
        ['sim'],
        mode='advanced',
        processes=processes,
    )
    plan = load_launch_sequence_plan(None, launches)

    assert plan['mode'] == 'advanced'
    assert plan['processes'] == processes


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


def test_absolute_bundled_launch_config_can_load_user_config_fallback(
    monkeypatch,
    tmp_path,
):
    launch_dir = tmp_path / 'launch_sequences'
    bundled_root = tmp_path / 'resources'
    monkeypatch.setattr(config_module, 'LAUNCH_SEQUENCE_DIR', launch_dir)
    monkeypatch.setattr(config_module, 'PROJECT_ROOT', bundled_root)
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
    bundled = bundled_root / 'config' / 'custom_auto.yaml'
    bundled.parent.mkdir(parents=True)
    bundled.write_text(
        '''
timeline:
  - at_seconds: 1
    button: old
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

    plan = load_launch_sequence_plan(buttons, bundled)

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


def test_bundled_docker_cp_config_has_no_default_copy_rows():
    config = load_docker_cp_config(Path('/missing/docker_cp_image_tag.yaml'))

    assert config == {}


def test_docker_cp_can_load_workspace_specific_user_config(monkeypatch, tmp_path):
    bundled = tmp_path / 'bundled.yaml'
    global_user = tmp_path / 'global.yaml'
    workspace_user = tmp_path / 'workspace.yaml'
    bundled.write_text(
        '''
default:
  host_to_container:
    - host: ~/Downloads/bundled.rviz
      container: /container/bundled.rviz
''',
        encoding='utf-8',
    )
    global_user.write_text(
        '''
default:
  host_to_container:
    - host: ~/Downloads/global.rviz
      container: /container/global.rviz
''',
        encoding='utf-8',
    )
    workspace_user.write_text(
        '''
default:
  host_to_container:
    - host: ~/Downloads/workspace.rviz
      container: /container/workspace.rviz
''',
        encoding='utf-8',
    )
    monkeypatch.setattr(config_module, 'DOCKER_CP_CONFIG_FILE', bundled)
    monkeypatch.setattr(config_module, 'USER_DOCKER_CP_CONFIG_FILE', global_user)

    config = load_docker_cp_config(workspace_user)

    assert config['default']['host_to_container'] == [
        {
            'container': '/container/workspace.rviz',
            'host': '~/Downloads/workspace.rviz',
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


def test_save_docker_cp_config_accepts_workspace_path(tmp_path):
    target = tmp_path / 'docker_cp_profiles' / 'gpt_ws_docker_cp_image_tag.yaml'

    saved = save_docker_cp_config(
        {
            'default': {
                'host_to_container': [
                    {
                        'host': '~/Downloads/source.rviz',
                        'container': '/container/source.rviz',
                    },
                ],
                'container_to_host': [],
            },
        },
        target,
    )

    assert saved == target
    assert target.is_file()


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
