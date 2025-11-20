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
- Modular ROS 2 backend:
  - Role-specific nodes: `MobilityNode`, `ScienceNode`, `ArmNode`, `CommsNode`
  - Generic Qt/ROS bridge + worker thread in `core/`

---

## 2. Repository Layout

```text
karura_gui/
├── README.md
├── requirements.txt
├── run/
│   ├── run_mobility.sh        # helper scripts to launch each dashboard
│   ├── run_science.sh
│   └── run_arm.sh
└── src/
    └── dashboard/
        ├── __init__.py
        ├── main_mobility.py   # entry point for Mobility GUI
        ├── main_science.py    # entry point for Science GUI
        ├── main_arm.py        # entry point for Arm GUI
        ├── core/              # shared Qt/ROS infrastructure
        │   ├── app.py         # Qt app bootstrap (creates bridge + window)
        │   ├── base_bridge.py # BaseROS2Bridge: generic Qt <-> ROS bridge
        │   ├── ros2_worker.py # QThread that runs rclpy.spin_once()
        │   ├── config.py      # topic names, role config, constants
        │   ├── logging_config.py
        │   └── styles.qss     # global Qt stylesheet
        ├── ros_backend/       # ROS 2 nodes for each dashboard role
        │   ├── __init__.py
        │   ├── base_node.py        # BaseDashboardNode with _dispatch()
        │   ├── mobility_node.py    # pubs/subs for mobility topics
        │   ├── arm_node.py         # pubs/subs for arm topics
        │   ├── science_node.py     # pubs/subs for science topics
        │   └── comms_node.py       # pubs/subs for comms/health topics
        ├── mobility/          # Mobility dashboard UI + bridge
        │   ├── __init__.py
        │   ├── bridge.py      # MobilityBridge (Qt signals + MobilityNode)
        │   ├── window.py      # MobilityMainWindow (layouts, widgets)
        │   ├── widgets.py     # reusable mobility-specific widgets
        │   └── view_model.py  # optional data models / adapters
        ├── science/           # Science dashboard UI + bridge
        │   ├── __init__.py
        │   ├── bridge.py      # ScienceBridge (Qt signals + ScienceNode)
        │   ├── window.py      # ScienceMainWindow
        │   ├── widgets.py
        │   └── view_model.py
        └── arm/               # Arm dashboard UI + bridge
            ├── __init__.py
            ├── bridge.py      # ArmBridge (Qt signals + ArmNode)
            ├── window.py      # ArmMainWindow
            ├── widgets.py
            └── view_model.py
