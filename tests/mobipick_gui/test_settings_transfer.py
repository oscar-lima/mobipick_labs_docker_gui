from pathlib import Path

import yaml

from mobipick_gui.settings_transfer import export_settings, import_settings
from mobipick_gui.workspaces import RosWorkspace, WorkspaceRegistry


def test_settings_round_trip_remaps_workspaces_and_embeds_profiles(tmp_path):
    source_master = tmp_path / 'old-home' / 'ros_ws'
    target_master = tmp_path / 'new-home' / 'ros_ws'
    source_config = tmp_path / 'old-config' / 'gui_settings.yaml'
    target_config = tmp_path / 'new-config' / 'gui_settings.yaml'
    profiles = tmp_path / 'source-profiles'
    buttons = profiles / 'buttons.yaml'
    launches = profiles / 'launches.yaml'
    buttons.parent.mkdir(parents=True)
    buttons.write_text('buttons:\n  - key: demo\n', encoding='utf-8')
    launches.write_text('timeline:\n  - button: demo\n', encoding='utf-8')
    source_config.parent.mkdir(parents=True)
    source_config.write_text(
        'terminal:\n  title: Imported Terminal\n',
        encoding='utf-8',
    )

    source_registry = WorkspaceRegistry(tmp_path / 'source-registry.yaml')
    source_registry.master_folder = str(source_master)
    source_registry.upsert(
        RosWorkspace(name='base_ws', path=str(source_master / 'base_ws'))
    )
    source_registry.upsert(
        RosWorkspace(
            name='demo_ws',
            path=str(source_master / 'nested' / 'demo_ws'),
            extends=['base_ws'],
            image='example/demo:latest',
            button_config=str(buttons),
            launch_config=str(launches),
            sim_command='roslaunch demo demo.launch',
        )
    )
    source_registry.active = 'demo_ws'
    bundle = tmp_path / 'settings.yaml'

    export_settings(
        bundle,
        source_registry,
        user_config_path=source_config,
    )

    exported = yaml.safe_load(bundle.read_text(encoding='utf-8'))
    serialized = bundle.read_text(encoding='utf-8')
    assert exported['format'] == 'mobipick-labs-docker-gui-settings'
    assert str(source_master) not in serialized
    assert str(buttons) not in serialized
    assert exported['workspace_registry']['active'] == 'demo_ws'

    target_registry = WorkspaceRegistry(tmp_path / 'target-registry.yaml')
    imported_profiles = tmp_path / 'new-config' / 'profiles'
    import_settings(
        bundle,
        target_registry,
        master_folder=target_master,
        user_config_path=target_config,
        profiles_dir=imported_profiles,
    )

    imported = target_registry.get('demo_ws')
    assert target_registry.active == 'demo_ws'
    assert imported.directory == target_master / 'nested' / 'demo_ws'
    assert imported.extends == ['base_ws']
    assert imported.image == 'example/demo:latest'
    assert imported.sim_command == 'roslaunch demo demo.launch'
    assert Path(imported.button_config).read_text(encoding='utf-8') == (
        buttons.read_text(encoding='utf-8')
    )
    assert Path(imported.launch_config).read_text(encoding='utf-8') == (
        launches.read_text(encoding='utf-8')
    )
    assert yaml.safe_load(target_config.read_text(encoding='utf-8')) == {
        'terminal': {'title': 'Imported Terminal'}
    }


def test_settings_import_rejects_unrelated_yaml(tmp_path):
    source = tmp_path / 'not-settings.yaml'
    source.write_text('hello: world\n', encoding='utf-8')
    registry = WorkspaceRegistry(tmp_path / 'registry.yaml')

    try:
        import_settings(
            source,
            registry,
            master_folder=tmp_path / 'ros_ws',
        )
    except ValueError as exc:
        assert 'not a Mobipick' in str(exc)
    else:
        raise AssertionError('Expected import_settings to reject the file')
