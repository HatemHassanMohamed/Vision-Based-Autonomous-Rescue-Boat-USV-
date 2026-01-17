# 🚀 Autonomous Rescue Boat (USV)

## 📌 Project Overview

The **Autonomous Rescue Boat (USV)** is a multidisciplinary engineering system that integrates **mechanical design, embedded electronics, and advanced control algorithms**.  
The platform is designed to autonomously navigate toward a target location, detect fire or thermal signals, and avoid obstacles in real time, making it suitable for **fire rescue, emergency response, and hazardous environments**.

The system is built around a **stable pontoon hull**, ensuring high buoyancy and minimal roll disturbance, while a sensor fusion–based navigation stack provides reliable guidance and control.

<img width="768" height="411" alt="Screenshot from 2026-01-17 02-14-45" src="https://github.com/user-attachments/assets/0950d4de-eb22-43b0-96cd-e2674272344b" />


---

## ✨ Key Features

- **Autonomous Navigation**
  - GPS-based waypoint tracking using **NEO-6M**
  - Local Cartesian mapping via equirectangular projection

- **Thermal Signal / Fire Detection**
  - **ESP32-CAM**–based thermal/fire detection
  - Achieves approximately **80% recognition accuracy**

- **Obstacle Avoidance**
  - **HC-SR04 ultrasonic sensors**
  - Real-time detection and avoidance within **20–350 cm**

- **Stability & Orientation Control**
  - **MPU-6050 IMU** for roll, pitch, and yaw estimation
  - Sensor fusion using Kalman filtering

- **Dual Thruster Propulsion**
  - Differential thrust steering
  - Controlled via **L298N motor driver**

---

## 🛠 Hardware Architecture

The system is centrally managed by an **Arduino Mega**, which coordinates sensing, navigation, control, and actuation.

| Component        | Function                                  | Communication |
|------------------|--------------------------------------------|---------------|
| Arduino Mega     | Main Controller                            | N/A           |
| GPS NEO-6M       | Real-time Location Acquisition              | UART          |
| MPU-6050 IMU     | 6-Axis Motion Tracking & Balance            | I2C           |
| ESP32-CAM        | Thermal Detection & Imaging                 | UART          |
| HC-SR04          | Obstacle / Collision Avoidance              | Digital       |
| L298N            | Dual Motor Speed & Direction Control        | PWM           |
| ESP32            | Communication & Data Handling               | UART          |
| LiPo Battery     | 7.4V 5200mAh Power Supply                   | DC            |

---

## 🏗 Mechanical Design

- **Hull Type:** Pontoon Hull  
  - Selected for enhanced stability and reduced water displacement

- **Dimensions:**  
  - Length: **522.5 mm**  
  - Width: **277.9 mm**  
  - Height: **60.5 mm**

- **Material:**  
  - PVC (Density ≈ **1400 kg/m³**)

- **Center of Mass:**  
  - X = **9.14 in**  
  - Y = **4.24 in**  
  - Z = **0.88 in**

---

## 🧠 Software & Algorithms

### 🔹 Sensor Fusion (Kalman Filter)

A **Simple Kalman Filter** is used to fuse:
- GPS position data
- IMU acceleration and angular velocity data  

This significantly reduces noise and estimation uncertainty, resulting in smoother trajectory tracking and heading estimation.

---

### 🔹 Guidance & Control (PID + LOS)

#### 1️⃣ Coordinate Transformation
- GPS latitude and longitude are converted to **local XY coordinates** using an **equirectangular projection**, simplifying motion planning and control.

#### 2️⃣ Line of Sight (LOS) Guidance
- Computes the required heading angle toward the target waypoint.

#### 3️⃣ PID Controller
- Controls differential motor speed based on heading error.

**PID Parameters:**
- `Kp = 2`
- `Ki = 2`
- `Kd = 0.2`

<img width="509" height="251" alt="image54" src="https://github.com/user-attachments/assets/f1bae4bd-8769-4421-88e2-7efd623355f9" />

<img width="774" height="322" alt="image88" src="https://github.com/user-attachments/assets/9c2a3ede-d667-4b18-b405-ee7facc8ad34" />

---

## 🔄 System Flow

1. **Initialize**
   - Set origin GPS coordinates and target waypoint
   - Initialize sensors and controllers

2. **Sense**
   - Read GPS, IMU, ultrasonic, and camera data

3. **Think**
   - Apply Kalman filtering
   - Compute LOS heading
   - Detect obstacles and update path

4. **Act**
   - Adjust motor speeds using PID output
   - Perform differential steering


---

## 📊 Performance Evaluation

- **Operational Endurance:**  
  - ~**68 minutes** on a single LiPo battery charge

- **Fire Detection Accuracy:**  
  - ~**80%** using trained detection models

- **Navigation Precision:**  
  - Obstacle avoidance and trajectory tracking refined using  
    **MATLAB modeling and dynamic simulation**

---

## 👥 Contributors

Developed by the **Mechatronics Project Team**  
Faculty of Engineering, Assiut University — **May 2024**

- Ibrahim Makkawy  
- Ahmed Yosri  
- Hanya Abdelkareem  
- Samira Ahmed  
- Beshoy Nasem  
- **Hatem Safwat**  
- Omar Khaled  
- Mostafa Naser  

**Supervised by:**  
**Dr. Gamal Abdel Nasser**

---

## 📜 License

This project is developed for academic and research purposes.  
For reuse or collaboration, please contact the authors.
