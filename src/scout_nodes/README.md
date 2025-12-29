# Scout & Delivery Drone System

This package (`scout_nodes`) implements an autonomous multi-drone system using ROS Noetic. A **Scout Drone** scans an area to detect targets (people) and identifies their GPS coordinates. A **Delivery Drone** then receives optimal routes to deliver payloads to these targets.

## 🚀 Key Features

### 1. Autonomous Area Scan (Lawnmower)
- **KML Input**: Reads a KML file (`config/search_area.kml`) to define the search boundary.
- **Polygon Grid**: Automatically generates a "Lawnmower" (zigzag) flight path inside the polygon.
- **Node**: `decision_node_scout.py`

### 2. Intelligent Rescheduling
- **Proximity Sorting**: All detected targets are dynamically sorted by their distance to the Delivery Drone.
- **Optimization**: Ensures the drone always visits the closest target NEXT, minimizing flight time.
- **Node**: `geolocation_node.py`

### 3. Duplicate Detection (Spatial Filtering)
- **Clustering**: Prevents detecting the same person multiple times.
- **Logic**: If a new detection is within **3.0m** of a known target, it is merged (averaged) to improve GPS accuracy instead of creating a duplicate.

### 4. MAVLink Integration
- **Standard Protocol**: No custom UDP packets. The system speaks native MAVLink.
- **Telemetry**: Transmits detections to Ground Control Stations (QGroundControl/Mission Planner) as `NAMED_VALUE_FLOAT`.
- **Node**: `communication_node_scout.py`

### 5. Verified Delivery
- **Safety**: The delivery drone actively checks telemetry (MAVLink `GLOBAL_POSITION_INT`) and only drops the payload when within **0.5m** of the target.

---

## 🛠️ Installation & Dependencies

### ROS Packages
Ensure you have the standard ROS Noetic desktop full installation.

```bash
sudo apt-get install ros-noetic-geographic-msgs ros-noetic-mavros ros-noetic-mavros-msgs
```

### Python Dependencies
Using `pip` for Python 3 (ROS Noetic standard):

```bash
pip3 install pymavlink shapely ultralytics numpy
```

---

## 🏃 Usage Guide

### 1. Configuration
- **Search Area**: Place your KML boundary file at `src/scout_nodes/config/search_area.kml`.
- **IP Addresses**: Update `communication_node_scout.py` if your Base Station IP matches `192.168.1.100`.

### 2. Build
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 3. Launch Nodes
Run the system nodes (start `roscore` first):

```bash
# 1. Start the Perception & Geolocation (The "Eyes")
rosrun scout_nodes geolocation_node.py

# 2. Start the Communication Link (The "Radio")
rosrun scout_nodes communication_node_scout.py

# 3. Start the Autonomy/Search Logic (The "Brain")
rosrun scout_nodes decision_node_scout.py

# 4. Start the Delivery Commander (The "Worker")
rosrun scout_nodes delivery_commander_node.py
```

### 4. Monitor with QGroundControl
1.  Open QGC on your Base Station.
2.  It will auto-connect to the Scout's MAVLink stream (UDP 14550).
3.  Go to **Analyze Tools -> Values**.
4.  Watch `TGT_COUNT` increase as targets are found!

---

## 🏗️ Architecture
- **Message Types**: Custom `TargetCoordinatesArray` handles lists of GPS points.
- **Coordinate Frame**: Uses WGS84 (Lat/Lon) for all inter-drone communication.
- **Fail-safes**: Sampled timeout (60s) for delivery verification.

## 📝 Troubleshooting
- **Missing `TargetCoordinatesArray`**: Did you run `catkin_make` and `source devel/setup.bash`?
- **No MAVLink**: Check your firewall and `BASE_STATION_IP` in `communication_node_scout.py`.
- **Duplicate Detections**: Adjust `DUPLICATE_THRESHOLD_M` in `geolocation_node.py` (Default: 3.0m).
