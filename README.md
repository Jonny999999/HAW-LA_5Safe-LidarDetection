# 5Safe Lidar Room-Scale LiDAR People Detection – HAW-LA

This repository contains the full system for **5Safe Room-Scale LiDAR Detection**, developed in the **IoT Project module** at **HAW Landshut**.

It is loosely inspired by the original *5Safe* project, which focused on outdoor/street-level LiDAR detection. Our version is an independent system tailored for **indoor room-scale environments**, using the same class of 3D LiDAR sensors in a dual-sensor setup.  
<br>

The Idea was to explore two detection strategies:
- An AI-based approach
- A custom detection method based on a simple algorithm for motion analysis

---

## System Overview

The core of this repository is the `lidar_system/` Python project. It integrates all key components:

- Real-time processing of LiDAR data from 1–2 sensors
- Motion-based detection using clustering and filtering
- Optional AI-based bounding box detection using PointNet
- Live visualization via Open3D (multiple synchronized windows)
- Merging of sensor views using transformation matrices
- TCP streaming for dashboards or external consumers
- Utilities for cropping, picking points, and exporting results

### Architecture Overview

![System Overview](doc/diagrams/overview.png)

---

## Screenshots and Visual Results

| Motion Detection (Custom) | Dual Sensor Merge |
|---------------------------|-------------------|
| ![Motion](doc/screenshots/py_motion-detection.jpg) | ![Merge](doc/screenshots/py_sensor-merge-utility_2.jpg) |

| AI vs. Algorithm Comparison |
|-----------------------------|
| ![Comparison](doc/diagrams/comparison_ai-vs-algorithm-output.png) |

---

## Repository Structure

```
lidar_system/              ← Main Python system (core logic, live detection)
data/                      ← Test recordings (.pcap, .mp4)
ai-detection/              ← AI experiments and detection (e.g. PointNet)
dashboard/                 ← TCP dashboard receiver examples
scripts/                   ← Utilities (e.g. transformation matrix tool, launchers)
legacy_projects/           ← Earlier standalone and single-sensor prototypes
doc/                       ← Documentation and screenshots
```

---

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/yourname/5Safe-LidarDetection.git
cd 5Safe-LidarDetection
git lfs pull
```

> **Note:** Git LFS is required to download recorded `.pcap` test files in the `data/` folder.

#### Install Git LFS if Needed:
* **Ubuntu/Debian:**
  ```bash
  sudo apt update
  sudo apt install git-lfs
  cd HAW-LA_5Safe-LidarDetection
  git lfs install
  ```
* **Windows (PowerShell):**
Download and install git-lfs from official the website and run:
  ```powershell
  cd HAW-LA_5Safe-LidarDetection
  git pull
  ```

---

### 2. Python Environment Setup
> **Note:** This project was developed and tested with **Python 3.10**. Other versions may not work correctly.

You can choose one of two options:
#### Option A: Global Installation (recommended for stable environments)
Ensure you are using Python 3.10:
```bash
sudo apt install python3.10 python3.10-venv python3-pip  # Ubuntu
python3.10 --version
```

Install the required packages:
```bash
pip3 install -r lidar_system/requirements.txt
```
If there are compatibility issues, try the extended requirements:
```bash
pip3 install -r lidar_system/requirements_full.txt
```

#### Option B: Virtual Environment (safer, isolated setup)

Create and activate a virtual environment:
```bash
python3.10 -m venv ~/python-envs/py3.10-5Safe
source ~/python-envs/py3.10-5Safe/bin/activate
```

Then install dependencies:
```bash
pip install -r lidar_system/requirements.txt
```

If issues occur:
```bash
pip install -r lidar_system/requirements_full.txt
```

---

## Usage

### Basic Launch

Run from terminal:

```bash
./scripts/start_python-script+stats-terminal.sh
```

This starts the main detection system and a secondary terminal showing live output.

Or manually:

```bash
cd lidar_system
python main.py
```

---

## Internal Python Architecture

The internal structure of the python project is based on multiple parallel threads or processes communicating via queues:
![Python Threads](doc/diagrams/python_threads.png)

---

## Modes of Operation

### A) Using recorded PCAP Test Data (Offline - No Sensors Required)

1. Open `lidar_system/config.py`
2. Set `USE_LIVE_UDP = False`
3. Comment in a `.pcap` file path from `data/testdata/`
4. Run the system

### B) Using Real Dual-Sensor Setup

1. Connect two sensors via Ethernet
2. Use Wireshark to find their current IPs
3. Access their config pages in a browser:

   ![Web Interface](doc/screenshots/velodyne-webinterface.jpg)

   - Change IPs to your desired subnet
   - Set destination IP to the server’s IP
   - Assign unique UDP ports per sensor

4. Run `tcpdump` to confirm receipt:

```bash
sudo tcpdump -i enp71s0f0np0 udp port 5001
```

5. Update `config.py`:
   - Set `USE_LIVE_UDP = True`
   - Adjust UDP port, interface, and sensor modes

---

### Initial Sensor Merge Setup (Transformation matrix)

1. Mount sensors in the room (slightly angled down)
2. Set window modes to `sensor1` and `sensor2` in config.py
3. Use terminal commands `pick1` and later same with `pick2` to open point picker windows for the slected sensor
4. SHIFT+click to select 3 matching points in each view

   ![Point Picker](doc/screenshots/py_sensor-merge-utility_1.jpg)

5. Copy the points from terminal output
6. Manually edit the script `scripts/calculate_transformation_matrix.py`
Insert your points e.g.:

```python
points_pc1 = np.array([
    [-1.855261207, -7.051774979, 0.591801226],  # Greenscreen cabinets
    [2.209504128, -7.254121304, 0.439154744],   # Greenscreen center
    [2.961946249, -11.077330589, 0.465429336],  # Greenscreen door
])

points_pc2 = np.array([
    [-5.001546383, -6.413216591, 0.660472870],  # Greenscreen cabinets
    [-2.166234493, -3.555836439, -0.047979854], # Greenscreen center
    [1.149047494, -5.592765808, 0.164931178],   # Greenscreen door
])
```
6. Calculate the matrix, Run:
```bash
scripts/calculate_transformation_matrix.py
```
7. Paste the resulting matrix into `config.py` under `TRANSFORMATION_MATRIX`.

8. Verify the merge: Set one window to `merged_dual` mode to verify alignment.

---

### Point Cloud Cropping (2D Polygon)
To remove walls/floor, define a polygon that bounds the area of interest in `config.py` e.g.:
```python
CROP_POINTCLOUD_POLYGON = [
    (0.0696, -0.1957),        
    (-4.188, -5.236),         
    (2.691, -11.679),         
    (7.267, -6.364)           
]
```

---

## Helper Scripts

- `scripts/calculate_transformation_matrix.py`: Generate merge matrix from point pairs
- `scripts/start_python-script+stats-terminal.sh`: Auto-restarts GUI and launches status window
- `scripts/watch_status_json_file.sh`: Shows detection/debug output in terminal
- `dashboard/`: TCP receiver for status + cluster data
- `ai-detection/`: Standalone model experiments
- `legacy_projects/`: One-off tests and early motion detection prototypes

---