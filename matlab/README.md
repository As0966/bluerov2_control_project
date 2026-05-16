# MATLAB Analysis Suite: BlueROV2 4-DOF Control

This directory contains the MATLAB tools for evaluating the performance of various control architectures implemented for the **BlueROV2-Inspired 4-DOF Underwater Vehicle**.

## Project Title
> **Mathematical Modeling and Controller Design for a BlueROV2-Inspired 4-DOF Underwater Vehicle**

---

## 🌊 Mathematical Model (4-DOF)

The vehicle is modeled in 4 Degrees of Freedom (DOF): **Surge, Sway, Heave, and Yaw**. The nonlinear dynamics are described by the standard Fossen's equation:

$$M \dot{\nu} + C(\nu)\nu + D(\nu)\nu + g(\eta) = \tau$$

Where:
- $\eta = [x, y, z, \psi]^T$ is the position and orientation in the inertial frame.
- $\nu = [u, v, w, r]^T$ is the velocity vector in the body-fixed frame.
- $M$ is the inertia matrix (including added mass).
- $C(\nu)$ is the Coriolis and centripetal matrix.
- $D(\nu)$ is the damping matrix (linear and quadratic).
- $g(\eta)$ is the vector of gravitational and buoyancy forces.
- $\tau$ is the vector of control inputs (forces and moments).

---

## 🛠️ Features of the Suite

### 1. Multi-Controller Evaluation
The suite processes data from several advanced control strategies:
- **MRAC**: Model Reference Adaptive Control.
- **Adaptive Backstepping**: Lyapunov-based nonlinear control with parameter estimation.
- **Feedback Linearization (FL)**: Input and Input-Output linearization techniques.
- **Inverse Neural Network (INN)**: Adaptive neural control for compensation of uncertainties.
- **State Feedback**: Standard linear control for baseline comparison.

### 2. Trajectory Analysis
Visualizes performance across three standard mission profiles:
- **Circle**: Constant curvature tracking.
- **Figure-8 (Lemniscate)**: Complex maneuvering and turning.
- **Spline**: General path following.

### 3. Metric Quantification
Calculates and visualizes standard error metrics:
- **RMSE**: Root Mean Square Error.
- **MAE**: Mean Absolute Error.
- **ITAE**: Integral Time-Absolute Error.

---

## 🚀 How to Use

1. **Prerequisites**: Ensure you have MATLAB (R2020b or later recommended) installed.
2. **Data**: The script expects `.csv` results to be present in the `../results/` directory.
3. **Run**:
   - Open MATLAB.
   - Navigate to the `matlab/` directory.
   - Run the script: `Analyze_BlueROV2`.
4. **Output**:
   - High-resolution figures will be saved in the `./figures/` subdirectory.
   - A summary table of performance metrics will be printed to the Command Window.

---

## 👤 Author
**Bayisa Ligaba Janka**  
*Electrical Electronics Engineer*  
GitHub: [@As0966](https://github.com/As0966)

---
*Developed for the HASTEN-AUV Project (TÜBİTAK 1002A).*
