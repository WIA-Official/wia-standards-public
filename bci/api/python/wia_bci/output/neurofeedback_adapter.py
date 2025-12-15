"""
WIA BCI Neurofeedback Adapter

Real-time brain activity visualization (console-based mock).
"""

from typing import Optional, List, Dict, Any
import time

from .base_output import BaseOutputAdapter
from .types import (
    OutputType,
    OutputContent,
    OutputOptions,
    VisualizationMode,
    ChannelData,
    CursorPosition,
    NeurofeedbackTheme,
    ClassificationContent,
)


class NeurofeedbackAdapter(BaseOutputAdapter):
    """Neurofeedback Adapter (console-based mock)."""

    def __init__(self):
        super().__init__()
        self._mode: VisualizationMode = "band_powers"
        self._theme = NeurofeedbackTheme()
        self._band_powers: Dict[str, float] = {}
        self._channels: List[ChannelData] = []
        self._classification: Optional[ClassificationContent] = None
        self._cursor = CursorPosition(x=0, y=0)
        self._power_history: List[Dict[str, float]] = []
        self._max_history = 100

    @property
    def type(self) -> OutputType:
        return "neurofeedback"

    @property
    def name(self) -> str:
        return "Console Neurofeedback"

    async def initialize(self, options: Optional[OutputOptions] = None) -> None:
        """Initialize adapter."""
        self._ready = True
        self._emit("ready")

    def set_visualization(self, mode: VisualizationMode) -> None:
        """Set visualization mode."""
        self._mode = mode

    def get_visualization(self) -> VisualizationMode:
        """Get visualization mode."""
        return self._mode

    def set_theme(self, theme: NeurofeedbackTheme) -> None:
        """Set theme."""
        self._theme = theme

    def update_band_powers(self, powers: Dict[str, float]) -> None:
        """Update band powers."""
        self._band_powers = powers
        self._power_history.append(powers)
        if len(self._power_history) > self._max_history:
            self._power_history.pop(0)

    def update_topography(self, channels: List[ChannelData]) -> None:
        """Update topography."""
        self._channels = channels

    def update_classification(self, result: ClassificationContent) -> None:
        """Update classification."""
        self._classification = result

    def update_cursor(self, position: CursorPosition) -> None:
        """Update cursor position."""
        self._cursor = position

    async def output(self, content: OutputContent) -> None:
        """Output content."""
        if content.type == "signal" and content.signal:
            self.update_band_powers(content.signal.band_powers)
            self.update_topography(content.signal.channels)
            self._render()
        elif content.type == "classification" and content.classification:
            self.update_classification(content.classification)
            self._render()

    def _render(self) -> None:
        """Render to console (simplified)."""
        if self._mode == "band_powers":
            self._render_band_powers()
        elif self._mode == "classification":
            self._render_classification()
        elif self._mode == "cursor":
            self._render_cursor()

    def _render_band_powers(self) -> None:
        """Render band powers."""
        if not self._band_powers:
            return

        bands = ["delta", "theta", "alpha", "beta", "gamma"]
        output = "Band Powers: "
        for band in bands:
            value = self._band_powers.get(band, 0)
            bar_len = int(min(20, value / 5))
            bar = "█" * bar_len
            output += f"{band[0].upper()}: {bar:20s} ({value:.1f}) | "

        print(f"\r{output}", end="", flush=True)

    def _render_classification(self) -> None:
        """Render classification."""
        if not self._classification:
            print("\rWaiting for classification...", end="", flush=True)
            return

        conf = self._classification.confidence
        bar_len = int(conf * 20)
        bar = "█" * bar_len + "░" * (20 - bar_len)

        print(
            f"\r[{self._classification.class_name.upper():15s}] "
            f"{bar} ({conf * 100:.0f}%)",
            end="",
            flush=True,
        )

    def _render_cursor(self) -> None:
        """Render cursor position."""
        x_pos = int((self._cursor.x + 1) * 10)
        y_pos = int((self._cursor.y + 1) * 5)
        click = "●" if self._cursor.click else "○"

        print(
            f"\rCursor: ({self._cursor.x:+.2f}, {self._cursor.y:+.2f}) {click}",
            end="",
            flush=True,
        )

    async def dispose(self) -> None:
        """Dispose adapter."""
        print()  # New line after console output
        await super().dispose()
