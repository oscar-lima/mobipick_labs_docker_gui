import sys

from mobipick_gui import cli


def test_create_application_suppresses_only_known_startup_warning(monkeypatch, capsys):
    monkeypatch.setattr(cli, 'session_type', lambda: 'x11')
    monkeypatch.setattr(cli, 'QIcon', lambda path: path)
    installed_handlers = []
    forwarded = []

    def previous_handler(message_type, context, message):
        forwarded.append((message_type, context, message))

    def install_handler(handler):
        installed_handlers.append(handler)
        return previous_handler if len(installed_handlers) == 1 else None

    class FakeApplication:
        def __init__(self, arguments):
            handler = installed_handlers[-1]
            handler(1, None, cli._QT_SOCKET_NOTIFIER_THREAD_WARNING)
            handler(1, None, cli._QT_WAYLAND_ACTIVATION_WARNING)
            handler(1, None, 'another Qt warning')
            self.arguments = arguments

        def setWindowIcon(self, icon):
            self.icon = icon

    monkeypatch.setattr(cli, 'qInstallMessageHandler', install_handler)
    monkeypatch.setattr(cli, 'QApplication', FakeApplication)

    app = cli._create_application(['mobipick-gui'])

    assert app.arguments == ['mobipick-gui']
    assert app.icon == str(cli._APPLICATION_ICON)
    assert cli.QCoreApplication.applicationName() == (
        cli._APPLICATION_DESKTOP_ID
    )
    assert forwarded == [
        (1, None, cli._QT_WAYLAND_ACTIVATION_WARNING),
        (1, None, 'another Qt warning'),
    ]
    assert installed_handlers[-1] is previous_handler
    assert capsys.readouterr().err == ''


def test_create_application_prints_other_messages_without_previous_handler(
    monkeypatch,
    capsys,
):
    monkeypatch.setattr(cli, 'session_type', lambda: 'x11')
    monkeypatch.setattr(cli, 'QIcon', lambda path: path)
    installed_handlers = []

    def install_handler(handler):
        installed_handlers.append(handler)
        return None

    class FakeApplication:
        def __init__(self, _arguments):
            installed_handlers[-1](1, None, 'important Qt warning')

        def setWindowIcon(self, icon):
            self.icon = icon

    monkeypatch.setattr(cli, 'qInstallMessageHandler', install_handler)
    monkeypatch.setattr(cli, 'QApplication', FakeApplication)

    cli._create_application(['mobipick-gui'])

    assert capsys.readouterr().err == 'important Qt warning\n'


def test_create_application_filters_activation_warning_on_wayland(monkeypatch):
    monkeypatch.setattr(cli, 'session_type', lambda: 'wayland')
    monkeypatch.setattr(cli, 'QIcon', lambda path: path)
    installed_handlers = []
    forwarded = []

    def previous_handler(message_type, context, message):
        forwarded.append((message_type, context, message))

    def install_handler(handler):
        installed_handlers.append(handler)
        return previous_handler if len(installed_handlers) == 1 else None

    class FakeApplication:
        def __init__(self, arguments):
            self.arguments = arguments

        def setWindowIcon(self, icon):
            self.icon = icon

        def setDesktopFileName(self, name):
            self.desktop_file_name = name

        @staticmethod
        def platformName():
            return 'wayland'

    monkeypatch.setattr(cli, 'qInstallMessageHandler', install_handler)
    monkeypatch.setattr(cli, 'QApplication', FakeApplication)

    app = cli._create_application(['mobipick-gui'])
    runtime_handler = installed_handlers[-1]
    runtime_handler(1, None, cli._QT_WAYLAND_ACTIVATION_WARNING)
    runtime_handler(1, None, 'another Qt warning')

    assert app.arguments == ['mobipick-gui']
    assert app.desktop_file_name == cli._APPLICATION_DESKTOP_ID
    assert len(installed_handlers) == 1
    assert forwarded == [(1, None, 'another Qt warning')]


def test_create_application_trusts_actual_non_wayland_platform(monkeypatch):
    monkeypatch.setattr(cli, 'session_type', lambda: 'wayland')
    monkeypatch.setattr(cli, 'QIcon', lambda path: path)
    installed_handlers = []

    def install_handler(handler):
        installed_handlers.append(handler)
        return None

    class FakeApplication:
        def __init__(self, _arguments):
            pass

        def setWindowIcon(self, icon):
            self.icon = icon

        def setDesktopFileName(self, name):
            self.desktop_file_name = name

        @staticmethod
        def platformName():
            return 'offscreen'

    monkeypatch.setattr(cli, 'qInstallMessageHandler', install_handler)
    monkeypatch.setattr(cli, 'QApplication', FakeApplication)

    cli._create_application(['mobipick-gui'])

    assert installed_handlers == [installed_handlers[0], None]


def test_install_user_desktop_entry_matches_x11_application_id(tmp_path):
    launcher = tmp_path / 'checkout' / 'gui.py'

    desktop_file = cli._install_user_desktop_entry(
        environ={'XDG_DATA_HOME': str(tmp_path / 'data')},
        argv0=str(launcher),
    )

    assert desktop_file == (
        tmp_path
        / 'data'
        / 'applications'
        / 'mobipick-labs-docker-gui.desktop'
    )
    content = desktop_file.read_text(encoding='utf-8')
    assert 'Name=Mobipick Labs Control\n' in content
    assert f'Exec="{sys.executable}" "{launcher}"\n' in content
    assert f'Icon={cli._APPLICATION_ICON.resolve()}\n' in content
    assert 'StartupWMClass=mobipick-labs-docker-gui\n' in content
