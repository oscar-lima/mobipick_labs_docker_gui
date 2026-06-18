"""Package version helpers."""
from __future__ import annotations

try:
    from importlib.metadata import PackageNotFoundError, version
except ImportError:  # pragma: no cover - Python < 3.8 fallback
    from importlib_metadata import PackageNotFoundError, version  # type: ignore

PACKAGE_NAME = 'mobipick-labs-docker-gui'
_FALLBACK_VERSION = '0.1.0'


def get_version() -> str:
    """Return the installed package version, with a source-tree fallback."""
    try:
        return version(PACKAGE_NAME)
    except PackageNotFoundError:
        return _FALLBACK_VERSION


__version__ = get_version()


__all__ = ['PACKAGE_NAME', '__version__', 'get_version']
