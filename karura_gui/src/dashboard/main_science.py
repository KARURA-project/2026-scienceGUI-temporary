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

from pathlib import Path

def main():
    """
    Initialize and launch the Science Dashboard application.
    """
    # Create Qt application
    app = QApplication(sys.argv)
    qss_path = Path(__file__).resolve().parent / "core" / "karura_dark.qss"
    app.setStyleSheet(qss_path.read_text(encoding="utf-8"))
    print(f"[STYLE] Loaded QSS: {qss_path}", file=sys.stderr, flush=True)
    
    try:
        # Import here to catch missing ROS 2 modules gracefully
        from karura_gui.science import ScienceMainWindow, ScienceBridge
    except ImportError as e:
        if "sensor_msgs" in str(e) or "rclpy" in str(e) or "std_msgs" in str(e):
            msg = (
                "ROS 2 libraries (sensor_msgs, rclpy) not found.\n"
                "Please source your ROS 2 environment:\n"
                "  source /opt/ros/humble/setup.bash"
            )
            print(f"[ERROR] {msg}", file=sys.stderr)
            QMessageBox.critical(None, "ROS 2 Missing", msg)
            sys.exit(1)
        raise e

    try:
        print("[DEBUG] Initializing ScienceBridge...", file=sys.stderr, flush=True)
        # Initialize ROS 2 bridge and start worker thread
        bridge = ScienceBridge()
        print("[DEBUG] Starting ScienceBridge thread...", file=sys.stderr, flush=True)
        bridge.start()
        
        print("[DEBUG] Creating ScienceMainWindow...", file=sys.stderr, flush=True)
        # Create and show main window
        window = ScienceMainWindow()
        print("[DEBUG] Connecting signals...", file=sys.stderr, flush=True)
        window.connect_signals(bridge)
        print("[DEBUG] Showing window...", file=sys.stderr, flush=True)
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
