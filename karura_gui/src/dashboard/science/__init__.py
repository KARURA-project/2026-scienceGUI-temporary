"""
Science Dashboard Module

Provides the PySide6-based UI for the Karura Science Dashboard,
including telemetry displays, camera feeds, and sensor monitoring.
"""

from .window import ScienceMainWindow
from .bridge import ScienceBridge

__all__ = ["ScienceMainWindow", "ScienceBridge"]
