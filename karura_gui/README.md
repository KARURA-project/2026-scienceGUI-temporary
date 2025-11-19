# Karura Rover Dashboards (PySide6 + ROS 2)

Multi-dashboard control and telemetry GUI for the Karura URC rover, built with **PySide6** and **ROS 2**.

This repository provides three separate dashboards:

- **Mobility Dashboard** – main driving HUD and high-level rover status
- **Science Dashboard** – science sensor telemetry and experiment controls
- **Arm Dashboard** – robotic arm joint states, commands, and camera views

Each dashboard runs as a separate application (typically on separate laptops) but shares a common backend for ROS 2 integration, configuration, and styling.

---

## 1. Features (Initial Scope)

- PySide6-based desktop UI (Qt Widgets)
- ROS 2 `rclpy` backend running in a background thread
- Qt signal/slot bridge from ROS 2 callbacks to GUI widgets
- Shared styles, logging, and configuration across all dashboards
- Separate entry points per dashboard:
  - `mobility` – main driving and battery HUD
  - `science` – science sensor and experiment panel
  - `arm` – manipulator control and feedback

---

## 2. Repository Layout

```text
karura_gui/
├── README.md
├── pyproject.toml          # or setup.cfg + requirements.txt
├── requirements.txt
├── run/
│   ├── run_mobility.sh
│   ├── run_science.sh
│   └── run_arm.sh
└── src/
    └── dashboard/
        ├── __init__.py
        ├── mobility_main.py
        ├── main_science.py
        ├── main_arm.py
        ├── core/
        │   ├── app.py              # Qt app bootstrap
        │   ├── ros2_bridge.py      # ROS 2 <-> Qt bridge (Node + QThread)
        │   ├── config.py           # topic names, role config, etc.
        │   ├── logging_config.py
        │   └── styles.qss
        ├── mobility_main/
        │   ├── __init__.py
        │   ├── window.py           # MobilityMainWindow
        │   ├── widgets.py
        │   └── view_model.py
        ├── science/
        │   ├── __init__.py
        │   ├── window.py           # ScienceMainWindow
        │   ├── widgets.py
        │   └── view_model.py
        └── arm/
            ├── __init__.py
            ├── window.py           # ArmMainWindow
            ├── widgets.py
            └── view_model.py
