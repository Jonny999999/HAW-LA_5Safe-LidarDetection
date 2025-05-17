# 5Safe Lidar Detection – HAW-LA

This repository contains the full project for "5Safe Lidar Detection", developed as part of the **IoT-Project module** at **HAW Landshut**.

<br>
The goal is to build on the existing **5Safe project**, which originally focused on detecting people in outdoor or street-like environments. Our extension adapts this approach to an **indoor room-scale** setting using multiple 3D LiDAR sensors.

<br>
We explore two detection strategies:
- A pretrained AI-based approach using existing toolchains (e.g., KITTI-style datasets)
- A custom detection method based on a simple algorithm for motion analysis

<br>
The project also includes visualization, post-processing, and tools for analyzing and exporting results.

## Repository Structure

- `doc/`: Project documentation, diagrams, and reports
- `data/`: Recorded sensor data and test cases
- `ai-detection/`: AI-based detection approaches
- `custom-detection/`: Custom logic and movement-based detection
- `output-processing/`: Scripts to merge and analyze results
- `dashboard/`: Visualization and interaction interface
- `scripts/`: Helper tools and conversion scripts


## Repo Setup / Installation

For the testdata files in the `data/` folder (like recorded LiDAR logs) to work, **Git LFS must be installed** — otherwise, you'll only get pointer files instead of real content.

### Install Git LFS:

- **Linux**:  
  `sudo pacman -S git-lfs && git lfs install`

- **Windows** (PowerShell):  
  `choco install git-lfs`  
  or download at https://git-lfs.com/  
  then  
  `git lfs install`

### After cloning the repo:
```bash
git lfs pull
```