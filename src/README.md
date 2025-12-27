# Scout Drone Perception & Mission System

A modular ROS Noetic package for autonomous search-and-rescue drones. This system integrates YOLOv8 based object detection, real-time geolocation of targets, and a complete mission control pipeline including ground station communication and delivery drone coordination.

## 📂 Project Structure

```text
catkin_ws/src/
├── custom_msgs/                  # Custom ROS Message Definitions
│   ├── msg/
│   │   ├── YoloDetection.msg     # Bounding box & class info
│   │   └── YoloDetectionArray.msg # Array of detections with header
│   ├── CMakeLists.txt
│   └── package.xml
│
├── scout_nodes/                  # Main Package
│   ├── launch/
│   │   ├── perception.launch     # Starts Sensors + YOLO + Geolocation
│   │   ├── sensors.launch        # Starts only hardware drivers
│   │   └── scout_mission.launch  # FULL SYSTEM: Perception + Comms + Logic
│   │
│   ├── scripts/
│   │   ├── camera_node.py           # Webcam/Video stream publisher
│   │   ├── thermal_camera_node.py   # Thermal camera driver/sim
│   │   ├── lidar_node.py            # LiDAR driver/sim
│   │   ├── yolo_node.py             # YOLOv8 Inference
│   │   ├── geolocation_node.py      # Target GPS calculation
│   │   ├── fake_drone_publisher.py  # Simulation data generator
│   │   ├── communication_node_scout.py # Drone-side telemetry bridge
│   │   ├── communication_node_base.py  # Base Station bridge (Laptop)
│   │   ├── decision_node_scout.py      # Mission State Machine
│   │   └── delivery_commander_node.py  # MAVLink Controller for 2nd drone
│   │
│   ├── CMakeLists.txt
│   └── package.xml
│
└── README.md
```

## 🚀 Features

- **Object Detection**: integrated YOLOv8 node (`yolo_node.py`) detecting persons from aerial video feeds.
- **Geolocation**: Real-time target coordinate estimation (`geolocation_node.py`) using drone GPS, IMU, and camera intrinsics.
- **Mission Logic**: Autonomous state machine for SEARCHING, TRACKING, and RTL modes.
- **Multi-Drone Control**: Can command a secondary delivery drone via MAVLink.
- **Telemetry Bridge**: UDP/TCP bridge for long-range communication between Drone and Base Station.

## 📋 Prerequisites

- **OS**: Ubuntu 20.04 (WSL2 supported)
- **ROS**: Noetic Ninjemys
- **Python Dependencies**:
  ```bash
  pip install ultralytics pyproj scipy numpy opencv-python pymavlink
  ```

## 🛠️ Installation & Replication

1.  **Create Workspace** (if you haven't already)
    ```bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    ```

2.  **Clone the Repository**
    ```bash
    git clone https://github.com/yourusername/scout_drone_system.git .
    ```

3.  **Build the Workspace**
    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

## 🏃 Usage / Testing

You can simulate the entire system on a single machine without hardware.

### 1. Launch the Scout Drone System
This starts the "Drone" side of things: sensors, perception logic, and mission control.
```bash
roslaunch scout_nodes scout_mission.launch
```
*Note: If no drone hardware is detected, it automatically starts `fake_drone_publisher` to simulate flight data.*

### 2. Launch the Base Station
Open a **new terminal** (source `devel/setup.bash` first) and run the ground station bridge. This mimics the laptop receiving data over WiFi.
```bash
rosrun scout_nodes communication_node_base.py
```

### 3. Verify System Output
Open a **third terminal** to monitor the critical topics:

*   **Target Coordinates** (Published when a person is seen):
    ```bash
    rostopic echo /mission/target_coordinates
    ```
*   **Mission State Commands**:
    ```bash
    rostopic echo /mission/command
    ```
*   **YOLO Detections**:
    ```bash
    rostopic echo /yolo/detections
    ```

## ⚙️ Configuration

- **IP Addresses**: Edit `scripts/communication_node_*.py` to set the IP of your Base Station and Drone.
- **Hardware**:
    - **Camera**: Edit `launch/sensors.launch` to change `camera_source` (default: 0 for webcam).
    - **LiDAR**: Edit `scripts/lidar_node.py` to set the serial port.
- **YOLO Model**: The system uses `yolov8n.pt`. It will auto-download on first run.

## 🤝 Contribution

1. Fork the Project
2. Create your Feature Branch
3. Commit your Changes
4. Push to the Branch
5. Open a Pull Request

## 📄 License

Distributed under the MIT License.
