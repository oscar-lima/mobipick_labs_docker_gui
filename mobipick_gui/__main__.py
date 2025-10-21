"""Allow ``python -m mobipick_gui`` to launch the GUI."""
from __future__ import annotations

import sys

from .cli import main

if __name__ == '__main__':
    sys.exit(main())
