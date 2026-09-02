"""Frame-synchronised robot animation for launch progress windows."""

from __future__ import annotations

import bisect
from pathlib import Path

from PyQt5.QtCore import Qt
from PyQt5.QtGui import QMovie
from PyQt5.QtWidgets import QLabel, QSizePolicy, QWidget


class RobotProgressAnimation(QLabel):
    """Display a GIF frame selected from an external progress fraction."""

    def __init__(
        self,
        gif_path: str | Path,
        parent: QWidget | None = None,
    ) -> None:
        super().__init__(parent)
        self.setAlignment(Qt.AlignCenter)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Fixed)

        self.movie = QMovie(str(gif_path), parent=self)
        self.movie.setCacheMode(QMovie.CacheAll)
        self._frame_starts: list[float] = []
        self._available = self._load_frames()
        if self._available:
            self.setFixedSize(self.movie.frameRect().size())
            self.setMovie(self.movie)
            self.movie.jumpToFrame(0)
        self.setVisible(self._available)

    @property
    def is_available(self) -> bool:
        """Return whether the GIF contains a usable animation."""
        return self._available

    def _load_frames(self) -> bool:
        """Decode the GIF and record each frame's normalized start time."""
        if not self.movie.isValid():
            return False
        self.movie.start()
        self.movie.setPaused(True)
        if not self.movie.jumpToFrame(0):
            return False

        frame_count = self.movie.frameCount()
        if frame_count <= 1:
            return False

        delays: list[int] = []
        for frame in range(frame_count):
            if not self.movie.jumpToFrame(frame):
                return False
            delay = self.movie.nextFrameDelay()
            delays.append(delay if delay > 0 else 10)

        total_delay = sum(delays)
        if total_delay <= 0:
            return False
        elapsed = 0
        for delay in delays:
            self._frame_starts.append(elapsed / total_delay)
            elapsed += delay
        self.movie.jumpToFrame(0)
        return True

    def set_progress(self, fraction: float) -> None:
        """Show the GIF frame corresponding to a zero-to-one fraction."""
        if not self._available:
            return
        bounded = min(1.0, max(0.0, float(fraction)))
        if bounded >= 1.0:
            frame = len(self._frame_starts) - 1
        else:
            frame = bisect.bisect_right(self._frame_starts, bounded) - 1
            frame = max(0, frame)
        if self.movie.currentFrameNumber() != frame:
            self.movie.jumpToFrame(frame)

    def reset(self) -> None:
        """Return the animation to its first frame."""
        self.set_progress(0.0)
