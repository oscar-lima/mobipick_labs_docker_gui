from mobipick_gui import cli


def test_create_application_suppresses_only_known_startup_warning(monkeypatch, capsys):
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
            handler(1, None, 'another Qt warning')
            self.arguments = arguments

    monkeypatch.setattr(cli, 'qInstallMessageHandler', install_handler)
    monkeypatch.setattr(cli, 'QApplication', FakeApplication)

    app = cli._create_application(['mobipick-gui'])

    assert app.arguments == ['mobipick-gui']
    assert forwarded == [(1, None, 'another Qt warning')]
    assert installed_handlers[-1] is previous_handler
    assert capsys.readouterr().err == ''


def test_create_application_prints_other_messages_without_previous_handler(
    monkeypatch,
    capsys,
):
    installed_handlers = []

    def install_handler(handler):
        installed_handlers.append(handler)
        return None

    class FakeApplication:
        def __init__(self, _arguments):
            installed_handlers[-1](1, None, 'important Qt warning')

    monkeypatch.setattr(cli, 'qInstallMessageHandler', install_handler)
    monkeypatch.setattr(cli, 'QApplication', FakeApplication)

    cli._create_application(['mobipick-gui'])

    assert capsys.readouterr().err == 'important Qt warning\n'
