# Backend 
The backend folder holds the functionality for the overall ROS2 publisher and subscriber methods. However, due to the nature of our gui maintaining multiple dashboards, the backend folder holds the shared infrastructure used by **all** dashboards. It is responsible for:

- Creating and running the Qt application
- Spinning ROS2 nodes in a background thread
- Providing a base Qt to ROS Bridge
- Central configuration, logging, and styling


## File Overview

```text
core/
├── app.py
├── base_bridge.py
├── ros2_worker.py
├── config.py
├── logging_config.py
└── styles.qss

### app.py:
(To be implemented) 

### 