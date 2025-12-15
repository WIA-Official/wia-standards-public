"""
WIA Cognitive AAC - Python ML Library
x¿ •` ¨©ê| \ Q AAC ! ®»

Mux ( ∫ì) - ¨ xD tmå X|
"""

__version__ = "1.0.0"
__author__ = "WIA Official"
__license__ = "MIT"

from .ml import SequenceModel, ContextModel
from .privacy import FederatedLearner

__all__ = [
    "SequenceModel",
    "ContextModel",
    "FederatedLearner",
]
