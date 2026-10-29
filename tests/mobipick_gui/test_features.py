import yaml

from mobipick_gui.features import (
    Preset,
    Selection,
    careful_turned_on,
    current_selection,
    gate_careful_options,
    features_path,
    load_features,
    load_presets,
    missing_options,
    parse_features,
    preset_problems,
    preset_values,
    save_preset,
    selection_values,
    without_careful,
)

FEATURES = {
    'version': 1,
    'groups': [
        {
            'name': 'Cable handling',
            'key': 'cable_handling',
            'kind': 'modes',
            'modes': [
                {'key': 'hack', 'label': 'Untangle poses', 'default': True, 'class': 'safe',
                 'sets': {'skip_untangle': 'false', 'pruning': 'false'}},
                {'key': 'cable_aware', 'label': 'Cable-aware', 'class': 'careful',
                 'sets': {'skip_untangle': 'true', 'pruning': 'true'}},
            ],
        },
        {
            'name': 'Open-set picks',
            'key': 'open_set_picks',
            'kind': 'features',
            'features': [
                {'key': 'jaw', 'label': 'Jaw along the box', 'class': 'safe',
                 'on': {'align_jaw': 'true'}, 'off': {'align_jaw': 'false'}},
                {'key': 'narrow', 'label': 'Narrow side first', 'class': 'safe', 'parent': 'jaw',
                 'on': {'narrow_side': 'true'}, 'off': {'narrow_side': 'false'}},
                {'key': 'padding', 'label': 'Box padding 0.5 cm', 'class': 'careful',
                 'on': {'box_padding': '0.005'}, 'off': {'box_padding': '0.01'}},
            ],
        },
    ],
}


def _features():
    loaded = parse_features(FEATURES)
    assert loaded.errors == []
    return loaded


def test_parse_keeps_groups_modes_and_features_in_order():
    loaded = _features()
    assert [group.key for group in loaded.groups] == ['cable_handling', 'open_set_picks']
    assert list(loaded.modes['cable_handling']) == ['hack', 'cable_aware']
    assert loaded.default_mode('cable_handling') == 'hack'
    assert loaded.features['padding'].careful
    assert loaded.options() == {'skip_untangle', 'pruning', 'align_jaw', 'narrow_side', 'box_padding'}


def test_parse_reports_problems_instead_of_raising():
    bad = {'groups': [
        {'name': 'M', 'kind': 'modes', 'modes': [{'key': 'a', 'sets': {}}, {'key': 'b', 'sets': {}}]},
        {'name': 'F', 'features': [{'key': 'x', 'on': {'a': '1'}, 'off': {}},
                                   {'key': 'y', 'parent': 'nope', 'on': {'b': 'true'}, 'off': {'b': 'false'}}]},
    ]}
    errors = parse_features(bad).errors
    assert any('exactly one mode' in error for error in errors)
    assert any('feature x' in error for error in errors)
    assert any('unknown parent nope' in error for error in errors)
    assert parse_features({'nothing': 1}).errors == ['the file needs a top-level "groups" list']


def test_selection_values_cover_every_option_and_a_child_needs_its_parent():
    loaded = _features()
    values = selection_values(loaded, Selection({'cable_handling': 'cable_aware'}, {'narrow', 'padding'}))
    assert values == {'skip_untangle': 'true', 'pruning': 'true', 'align_jaw': 'false', 'narrow_side': 'false',
                      'box_padding': '0.005'}


def test_preset_values_add_plain_options_and_default_modes():
    loaded = _features()
    preset = Preset(name='p', features_on=('jaw', 'narrow'), options={'cable_monitor': 'true'})
    assert preset_values(loaded, preset) == {
        'skip_untangle': 'false', 'pruning': 'false', 'align_jaw': 'true', 'narrow_side': 'true',
        'box_padding': '0.01', 'cable_monitor': 'true'}
    assert preset_problems(loaded, Preset(name='q', modes={'cable_handling': 'x', 'other': 'y'},
                                          features_on=('ghost',))) == [
        'unknown feature ghost', 'unknown mode cable_handling: x', 'unknown mode group other']


def test_current_selection_reads_the_values_back_and_marks_mixed_ones():
    loaded = _features()
    values = {'skip_untangle': 'true', 'pruning': 'true', 'align_jaw': 'true', 'narrow_side': 'false',
              'box_padding': '0.01'}
    selection, mixed = current_selection(loaded, values)
    assert selection.modes == {'cable_handling': 'cable_aware'}
    assert selection.features_on == {'jaw'}
    assert mixed == {}
    selection, mixed = current_selection(loaded, dict(values, pruning='false'))
    assert 'cable_handling' in mixed
    # an option the profile lacks is ignored: the mode still reads from the one that exists
    selection, mixed = current_selection(loaded, {'skip_untangle': 'true', 'align_jaw': 'false'})
    assert selection.modes == {'cable_handling': 'cable_aware'} and mixed == {}


def test_a_sim_default_mode_is_the_sims_fallback_and_the_real_robot_keeps_the_default():
    data = yaml.safe_load(yaml.safe_dump(FEATURES))
    data['groups'][0]['modes'][1]['sim_default'] = True
    loaded = parse_features(data)
    assert loaded.errors == []
    assert loaded.default_mode('cable_handling', sim=True) == 'cable_aware'
    assert loaded.default_mode('cable_handling') == 'hack'
    sim_values = preset_values(loaded, Preset(name='s', target='sim'))
    real_values = preset_values(loaded, Preset(name='r', target='real'))
    assert (sim_values['skip_untangle'], real_values['skip_untangle']) == ('true', 'false')
    assert current_selection(loaded, {}, sim=True)[0].modes == {'cable_handling': 'cable_aware'}
    assert current_selection(loaded, {})[0].modes == {'cable_handling': 'hack'}
    # the real robot drops a careful mode to the real robot's default, never to the sim's
    safe, _dropped = without_careful(loaded, Selection({'cable_handling': 'cable_aware'}, set()), allowed=set())
    assert safe.modes == {'cable_handling': 'hack'}
    data['groups'][0]['modes'][0]['sim_default'] = True
    assert parse_features(data).errors == ['group Cable handling: at most one mode may have "sim_default: true"']


def test_missing_options_name_what_the_profile_lacks():
    assert missing_options(_features(), {'skip_untangle', 'align_jaw', 'narrow_side'}) == {
        'padding': ['box_padding'], 'hack': ['pruning'], 'cable_aware': ['pruning']}


def test_careful_choices_go_off_for_the_real_robot_unless_allowed():
    loaded = _features()
    selection = Selection({'cable_handling': 'cable_aware'}, {'jaw', 'padding'})
    assert careful_turned_on(loaded, selection) == ['cable_aware', 'padding']
    safe, dropped = without_careful(loaded, selection, allowed={'padding'})
    assert dropped == ['cable_aware']
    assert safe.modes == {'cable_handling': 'hack'} and safe.features_on == {'jaw', 'padding'}


def test_plain_options_cannot_set_an_unticked_careful_value_back():
    loaded = _features()
    gated = selection_values(loaded, Selection({'cable_handling': 'hack'}, {'jaw'}))
    # a preset's or the remote API's plain options after the gated selection: careful values come back, others stay
    values = dict(gated, box_padding='0.005', skip_untangle='true', cable_monitor='true')
    final, refused = gate_careful_options(loaded, values, gated, allowed=set())
    assert final['box_padding'] == '0.01' and final['skip_untangle'] == 'false' and final['cable_monitor'] == 'true'
    assert set(refused) == {'box_padding', 'skip_untangle'}
    assert refused['box_padding'] == {'value': '0.005', 'reason': 'careful (padding): stays off on the real robot '
                                                                  'unless ticked in the Features dialog'}
    # a ticked careful feature keeps its value; an option the gated selection gives anyway is no refusal
    final, refused = gate_careful_options(loaded, dict(values, skip_untangle='false'), gated, allowed={'padding'})
    assert final['box_padding'] == '0.005' and refused == {}
    # an option no feature of the selection owns is left out rather than set
    final, refused = gate_careful_options(loaded, {'pruning': 'true'}, {}, allowed=set())
    assert final == {} and set(refused) == {'pruning'}


def test_features_and_presets_load_beside_the_profile_and_user_presets_win(tmp_path):
    profile = tmp_path / 'demo.yaml'
    profile.write_text('buttons: []\n')
    assert features_path(profile) is None and not load_features(profile)
    (tmp_path / 'demo_features.yaml').write_text(yaml.safe_dump(FEATURES))
    assert set(load_features(profile).features) == {'jaw', 'narrow', 'padding'}
    shipped = tmp_path / 'demo_presets'
    shipped.mkdir()
    (shipped / 'sim-full.yaml').write_text(yaml.safe_dump({'name': 'sim-full', 'features_on': ['jaw']}))
    (shipped / 'broken.yaml').write_text(yaml.safe_dump({'name': 'broken', 'target': 'moon'}))
    user = tmp_path / 'user'
    save_preset(Preset(name='sim-full', description='mine', features_on=('padding',)), user)
    save_preset(Preset(name='real / safe', target='real'), user)
    presets, errors = load_presets(profile, user)
    assert sorted(presets) == ['real / safe', 'sim-full']
    assert presets['sim-full'].user and presets['sim-full'].features_on == ('padding',)
    assert (user / 'real-safe.yaml').is_file()
    assert len(errors) == 1 and 'target must be sim or real' in errors[0]
