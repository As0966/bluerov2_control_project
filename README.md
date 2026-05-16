# Mathematical Modeling and Controller Design for a BlueROV2-Inspired 4-DOF Underwater Vehicle

[![MATLAB](https://img.shields.io/badge/Analysis-MATLAB-orange.svg)](./matlab)
[![ROS 2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange.svg)](https://gazebosim.org/docs/harmonic/install)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](./LICENSE)

This repository contains a comprehensive suite of advanced control algorithms for a BlueROV2-inspired underwater vehicle. Developed for research and benchmarking, it integrates **ROS 2 Humble** and **Gazebo Harmonic** with a professional **MATLAB Analysis Suite** for performance quantification.

---

## 🚀 Quick Start (Ubuntu 22.04)

### 1. Prerequisites
Ensure you have the following installed on your system:
- **Ubuntu 22.04 LTS** (Jammy Jellyfish)
- **ROS 2 Humble Desktop**
- **Gazebo Harmonic**
- **ROS-Gazebo Bridge**: `sudo apt install ros-humble-ros-gz`

### 2. Install Python Dependencies
```bash
pip install -r requirements.txt
```

### 3. Setup the Workspace
```bash
# Clone the repository
git clone https://github.com/As0966/bluerov2_control_project.git
cd bluerov2_control_project

# Build the ROS 2 workspace
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
cd ..
```

### 4. Launch Mission Control
The interactive Mission Control UI handles all simulation scenarios (Gazebo or Standalone).
```bash
./launch.sh
```

---

## 🧠 Available Controllers

| Controller | Category | Description |
| :--- | :--- | :--- |
| **Backstepping** | Robust | Lyapunov-based tracking with saturation compensation |
| **State Feedback** | Linear | Baseline PD-based 4-DOF state feedback |
| **Feedback Linearization** | Nonlinear | Computed-torque method with external PD loop |
| **Nonlinear Observer** | Observer-Based | Nonlinear Luenberger observer with Integral Bias Estimation |
| **Linear Observer** | Observer-Based | Linearized observer using 4-DOF rigid body model |
| **MRAC** | Adaptive | Model Reference Adaptive Control for unknown damping |
| **INN** | Neural | Inverse Neural Network trained via Backstepping Distillation |

---

## 📈 MATLAB Analysis Suite
The project includes a dedicated MATLAB suite for professional data analysis and visualization, ideal for generating publication-quality figures.

**Key Features:**
- 3D Trajectory tracking comparison.
- Error time-series analysis for all 4-DOF states.
- Control effort (thrust) evaluation.
- Automated performance metric quantification (RMSE, MAE, ITAE).

See the [MATLAB README](file:///home/bayisa/bluerov2_control_project/matlab/README.md) for usage instructions.

---

## 📊 Standalone Benchmarking
If you don't have Gazebo installed, you can still run high-fidelity simulations using the Python backend:
```bash
python3 src/simulate_observer.py --trajectory figure8 --duration 60
```
Results will be saved in `results/` (CSV) and `figures/` (PNG).

---

## 🛠️ Project Structure
- `ros2_ws/`: ROS 2 package containing the control node and thruster allocator.
- `src/`: Standalone Python simulation scripts for benchmarking.
- `matlab/`: Professional MATLAB Analysis Suite for performance quantification.
- `portfolio/`: High-resolution figures and research data.
- `docs/`: Technical documentation including the HASTEN-AUV proposal.

---

## 👤 Author
**Bayisa Ligaba Janka**  
*Electrical Electronics Engineer*  
GitHub: [@As0966](https://github.com/As0966)

---

## 📝 License
This project is for research and educational purposes. See `LICENSE` for details.
