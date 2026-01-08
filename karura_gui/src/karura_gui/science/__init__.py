"""
Re-export Science UI from the dashboard package so callers can:
    from karura_gui.science import ScienceMainWindow, ScienceBridge
while the real implementation lives under dashboard.science.
"""
from dashboard.science.window import ScienceMainWindow
from dashboard.science.bridge import ScienceBridge

__all__ = ["ScienceMainWindow", "ScienceBridge"]
