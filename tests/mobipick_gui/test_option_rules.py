import os

import pytest
import yaml

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from mobipick_gui.option_rules import (
    load_option_rules,
    option_rules_path,
    parse_option_rules,
)

from test_remote_ros_master import _create_window

REAL_ROBOT_RULES = {
    'rules': [
        {
            'when': {'remote_master': True},
            'only': {'world': ['cic_tables']},
            'reason': 'the real robot only runs cic_tables',
        }
    ]
}
WORLDS = {'world': ['moelk_tables', 'cic_tables']}


def test_only_rule_invalidates_other_options_while_condition_holds():
    rules = parse_option_rules(REAL_ROBOT_RULES)

    assert rules.invalid_options({'remote_master': False}, WORLDS) == {}
    assert rules.invalid_options({'remote_master': True}, WORLDS) == {
        'world': {'moelk_tables': 'the real robot only runs cic_tables'}
    }


def test_invalid_rule_matches_any_listed_condition_value():
    rules = parse_option_rules(
        {
            'rules': [
                {
                    'when': {'model_profile': ['o3', 'o3-jev']},
                    'invalid': {'disc_mode': 'cpu'},
                }
            ]
        }
    )
    choices = {'disc_mode': ['mockup', 'cpu']}

    assert rules.invalid_options({'model_profile': 'jev'}, choices) == {}
    invalid = rules.invalid_options({'model_profile': 'o3-jev'}, choices)
    assert list(invalid) == ['disc_mode']
    assert list(invalid['disc_mode']) == ['cpu']
    assert 'model_profile=o3|o3-jev' in invalid['disc_mode']['cpu']


def test_malformed_rules_are_reported_and_skipped():
    rules = parse_option_rules(
        {'rules': [{'when': {'remote_master': True}}, 'bad', REAL_ROBOT_RULES['rules'][0]]}
    )

    assert len(rules.rules) == 1
    assert len(rules.errors) == 2


def test_rules_file_is_found_beside_the_button_profile(tmp_path):
    profile = tmp_path / 'profile.yaml'
    profile.write_text('buttons: []\n')
    assert option_rules_path(profile) is None
    assert load_option_rules(profile).rules == []

    shared = tmp_path / 'option_rules.yaml'
    shared.write_text(yaml.safe_dump(REAL_ROBOT_RULES))
    assert option_rules_path(profile) == shared

    specific = tmp_path / 'profile_rules.yaml'
    specific.write_text(yaml.safe_dump(REAL_ROBOT_RULES))
    assert option_rules_path(profile) == specific
    assert len(load_option_rules(profile).rules) == 1


def test_remote_master_makes_moelk_tables_invalid(monkeypatch, tmp_path):
    app, window = _create_window(monkeypatch, tmp_path)
    try:
        window._option_rules = parse_option_rules(REAL_ROBOT_RULES)
        combo = window.world_combo
        combo.setCurrentIndex(combo.findText('moelk_tables'))
        moelk = combo.model().item(combo.findText('moelk_tables'))

        window.remote_master_checkbox.setChecked(True)

        assert window._current_world() == 'cic_tables'
        assert not moelk.isEnabled()
        assert 'cic_tables' in moelk.toolTip()
        world = next(e for e in window.generic_args() if e['name'] == 'world')
        assert list(world['invalid']) == ['moelk_tables']
        with pytest.raises(ValueError, match='invalid'):
            window.set_generic_args({'world': 'moelk_tables'})

        window.remote_master_checkbox.setChecked(False)

        assert moelk.isEnabled()
        window.set_generic_args({'world': 'moelk_tables'})
        assert window._current_world() == 'moelk_tables'
    finally:
        window.close()
        app.processEvents()
