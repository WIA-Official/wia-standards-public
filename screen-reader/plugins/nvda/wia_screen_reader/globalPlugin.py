"""
WIA Screen Reader - NVDA Global Plugin
"""
import globalPluginHandler
import api
import ui
import speech
import braille
import config
from scriptHandler import script

from .wihp_engine import WIHPEngine
from .braille_engine import WIABrailleEngine
from .config import WIAConfig


class GlobalPlugin(globalPluginHandler.GlobalPlugin):
    """
    WIA Screen Reader Global Plugin for NVDA.

    Provides WIHP pronunciation and WIA Braille conversion
    for enhanced accessibility.

    Keyboard Shortcuts:
        NVDA+Shift+W: Convert selection to WIHP
        NVDA+Shift+B: Convert selection to Braille
        NVDA+Shift+S: Speak with optimized TTS
        NVDA+Shift+L: Change language
    """

    def __init__(self):
        super().__init__()
        self.wihp = WIHPEngine()
        self.braille_engine = WIABrailleEngine()
        self.config = WIAConfig()
        self._language = "en"

    @script(
        description="Convert selected text to WIHP pronunciation",
        gesture="kb:NVDA+shift+w",
        category="WIA Screen Reader"
    )
    def script_convertToWIHP(self, gesture):
        """Convert selected text to WIHP and speak it."""
        text = self._getSelectedText()
        if not text:
            ui.message("No text selected")
            return

        result = self.wihp.convert(text, self._language)
        speech.speakMessage(result)
        ui.message(f"WIHP: {result}")

    @script(
        description="Convert selected text to Braille",
        gesture="kb:NVDA+shift+b",
        category="WIA Screen Reader"
    )
    def script_convertToBraille(self, gesture):
        """Convert selected text to Braille and display it."""
        text = self._getSelectedText()
        if not text:
            ui.message("No text selected")
            return

        result = self.braille_engine.convert(text)
        braille.handler.message(result.grade1)
        ui.message(f"Braille: {result.grade1}")

    @script(
        description="Speak selected text with optimized TTS",
        gesture="kb:NVDA+shift+s",
        category="WIA Screen Reader"
    )
    def script_speakOptimized(self, gesture):
        """Speak selected text with optimized pronunciation."""
        text = self._getSelectedText()
        if not text:
            ui.message("No text selected")
            return

        # Convert to WIHP for consistent pronunciation
        wihp_text = self.wihp.convert(text, self._language)
        speech.speakMessage(wihp_text)

    @script(
        description="Change WIA language",
        gesture="kb:NVDA+shift+l",
        category="WIA Screen Reader"
    )
    def script_changeLanguage(self, gesture):
        """Cycle through available languages."""
        languages = ["en", "ko", "ja", "zh", "es", "fr", "de"]
        current_index = languages.index(self._language) if self._language in languages else 0
        next_index = (current_index + 1) % len(languages)
        self._language = languages[next_index]

        language_names = {
            "en": "English",
            "ko": "Korean",
            "ja": "Japanese",
            "zh": "Chinese",
            "es": "Spanish",
            "fr": "French",
            "de": "German"
        }
        ui.message(f"Language: {language_names.get(self._language, self._language)}")

    @script(
        description="Show WIA Screen Reader info",
        gesture="kb:NVDA+shift+i",
        category="WIA Screen Reader"
    )
    def script_showInfo(self, gesture):
        """Show WIA Screen Reader information."""
        info = (
            "WIA Screen Reader v1.0.0\n"
            "Universal Accessibility Standard\n"
            "211 Languages Supported\n\n"
            "Shortcuts:\n"
            "NVDA+Shift+W: WIHP Pronunciation\n"
            "NVDA+Shift+B: Braille Conversion\n"
            "NVDA+Shift+S: Speak Optimized\n"
            "NVDA+Shift+L: Change Language\n\n"
            "弘益人間 - Benefit All Humanity"
        )
        ui.message(info)

    def _getSelectedText(self):
        """Get currently selected text."""
        try:
            obj = api.getFocusObject()
            if hasattr(obj, 'selection'):
                text = obj.selection
                if text:
                    return text.text if hasattr(text, 'text') else str(text)
        except Exception:
            pass

        # Try clipboard as fallback
        try:
            import api as nvda_api
            return nvda_api.getClipData()
        except Exception:
            pass

        return None

    def terminate(self):
        """Clean up when plugin is terminated."""
        super().terminate()
