# 🎼 Chorus
<img alt="Static Badge" src="https://img.shields.io/badge/build-passing-red?logo=github">


**Chorus** is a high-performance **Multi-DOF Online Trajectory Generator** designed for robotic applications that demand real-time, smooth, and synchronized motion planning — all while respecting system constraints. With built-in jerk limiting and trajectory tracking, Chorus delivers optimal motion profiles with ultra-low latency.

---

## ✨ Features

- 🧠 **Real-Time Online Generation**  
  Generates smooth, optimal trajectories on the fly for any number of degrees of freedom.

- ⏱️ **Fast Execution**  
  Optimized for real-time performance — computation time is **under 3 milliseconds**.

- 🔄 **Dynamic Target Updates**  
  Targets and constraints can be updated **during motion** without stopping the system.

- 📉 **Jerk-Limited Profiles**  
  Ensures continuous motion by limiting jerk, making trajectories smoother and safer for hardware.

- 🛠️ **Constraint-Aware Planning**  
  Respects user-defined limits on position, velocity, acceleration, and jerk.

- 🎯 **Synchronized Multi-DOF Motion**  
  All degrees of freedom are synchronized for coordinated and collision-free movement.

- 🚀 **Integrated Tracking Controller**  
  Includes a responsive trajectory tracking controller for **high tracking accuracy**.

---

## 🧩 Use Cases

- Industrial robot arms
- CNC machines
- Cobots in force-sensitive tasks
- Autonomous mobile robots
- Custom robotic platforms needing real-time planning

---

## 🚀 Getting Started

### Prerequisites

- C++17 or newer
- Eigen3
- CMake ≥ 3.10

### Build Instructions

```bash
git clone https://github.com/yourusername/Chorus.git
cd Chorus
mkdir build && cd build
cmake ..
make
```
