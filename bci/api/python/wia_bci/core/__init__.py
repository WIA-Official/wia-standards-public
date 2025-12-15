"""WIA BCI Core Classes."""

from wia_bci.core.wia_bci import WiaBci
from wia_bci.core.event_emitter import BciEventEmitter
from wia_bci.core.bci_error import BciError, ErrorCodes
from wia_bci.core.signal_processor import SignalProcessor

__all__ = [
    "WiaBci",
    "BciEventEmitter",
    "BciError",
    "ErrorCodes",
    "SignalProcessor",
]
