"""
Science Dashboard Entry Point

Launches the Karura Science Dashboard application.

Usage:
    python main_science.py

Environment:
    ROS 2 Jazzy must be sourced (i.e., 'source /opt/ros/jazzy/setup.bash')
"""

import sys
from PySide6.QtWidgets import QApplication, QMessageBox

from karura_gui.science import ScienceMainWindow, ScienceBridge


def main():
    """
    Initialize and launch the Science Dashboard application.
    """
    # Create Qt application
    app = QApplication(sys.argv)
    
    try:
        # Initialize ROS 2 bridge and start worker thread
        bridge = ScienceBridge()
        bridge.start()
        
        # Create and show main window
        window = ScienceMainWindow()
        window.connect_signals(bridge)
        window.show()
        
        # Cleanup on exit
        def on_exit():
            bridge.shutdown()
        
        app.aboutToQuit.connect(on_exit)
        
        # Run application
        sys.exit(app.exec())
    
    except Exception as e:
        error_msg = f"Failed to start Science Dashboard: {str(e)}"
        print(error_msg, file=sys.stderr)
        
        # Show error dialog
        QMessageBox.critical(None, "Science Dashboard Error", error_msg)
        sys.exit(1)


if __name__ == "__main__":
    main()
