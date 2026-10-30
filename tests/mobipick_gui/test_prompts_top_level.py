"""File and text prompts open as top-level windows (parent None): with a parent, GNOME attaches the prompt to that
window and dragging it drags the main window along, so the user cannot look behind it."""
import ast
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[2] / 'mobipick_gui'
PROMPTS = {'QFileDialog', 'QInputDialog'}


def prompt_calls():
    for path in sorted(PACKAGE.glob('*.py')):
        for node in ast.walk(ast.parse(path.read_text(), str(path))):
            func = getattr(node, 'func', None)
            if (isinstance(node, ast.Call) and isinstance(func, ast.Attribute) and func.attr.startswith('get')
                    and isinstance(func.value, ast.Name) and func.value.id in PROMPTS):
                yield f'{path.name}:{node.lineno}', node


def test_there_are_prompts_to_check():
    assert len(list(prompt_calls())) >= 20


def test_every_file_and_text_prompt_has_no_parent():
    with_parent = []
    for where, call in prompt_calls():
        parent = call.args[0] if call.args else next((k.value for k in call.keywords if k.arg == 'parent'), None)
        if not (isinstance(parent, ast.Constant) and parent.value is None):
            with_parent.append(where)
    assert with_parent == []


def blocking_dialogs():
    '''(where, constructor call) of every dialog run with exec_(); message boxes are covered by the pop-up helper'''
    for path in sorted(PACKAGE.glob('*.py')):
        for function in ast.walk(ast.parse(path.read_text(), str(path))):
            if not isinstance(function, ast.FunctionDef):
                continue
            built = {}
            for node in ast.walk(function):
                if (isinstance(node, ast.Assign) and isinstance(node.value, ast.Call) and len(node.targets) == 1
                        and isinstance(node.targets[0], ast.Name)):
                    built.setdefault(node.targets[0].id, []).append(node)
            for node in ast.walk(function):
                func = getattr(node, 'func', None)
                if not (isinstance(node, ast.Call) and isinstance(func, ast.Attribute) and func.attr == 'exec_'
                        and isinstance(func.value, ast.Name)):
                    continue
                before = [a for a in built.get(func.value.id, []) if a.lineno < node.lineno]
                if before and ast.unparse(before[-1].value.func).endswith(('Dialog', 'Wizard')):
                    yield f'{path.name}:{before[-1].lineno}', before[-1].value


def test_there_are_blocking_dialogs_to_check():
    assert len(list(blocking_dialogs())) >= 15


def test_every_blocking_dialog_has_no_parent():
    with_parent = []
    for where, call in blocking_dialogs():
        given = [k.value for k in call.keywords if k.arg == 'parent'] + [
            a for a in call.args if isinstance(a, ast.Name) and a.id in ('self', 'dialog')]
        if given and not all(isinstance(a, ast.Constant) and a.value is None for a in given):
            with_parent.append(where)
    assert with_parent == []


# their argument is the main window they work on, not a parent: they open without one
TAKES_THE_MAIN_WINDOW = {'FeaturesDialog'}


def windows_built_on_the_main_window():
    '''(where, call) of every dialog or wizard built with `self` as its parent, shown or run: a parent makes it a
    transient window that GNOME always stacks above the main window, so the user cannot put the main window in front'''
    for path in sorted(PACKAGE.glob('*.py')):
        for node in ast.walk(ast.parse(path.read_text(), str(path))):
            func = getattr(node, 'func', None)
            if not (isinstance(node, ast.Call) and isinstance(func, ast.Name) and func.id.endswith(('Dialog', 'Wizard'))
                    and func.id not in TAKES_THE_MAIN_WINDOW):
                continue
            given = [k.value for k in node.keywords if k.arg == 'parent'] + list(node.args)
            if any(isinstance(a, ast.Name) and a.id == 'self' for a in given):
                yield f'{path.name}:{node.lineno}'


def test_no_dialog_or_wizard_has_the_main_window_as_parent():
    assert list(windows_built_on_the_main_window()) == []
