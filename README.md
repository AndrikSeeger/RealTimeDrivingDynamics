# 🚗 RealTimeDrivingDynamics

## ⚙️ Overview

**RealTimeDrivingDynamics** is a high-efficiency, real-time ECU-style embedded system running on a Teensy 4.1 microcontroller. Designed for precise vehicle dynamics analysis, it uses **hardware interrupts** and **direct memory access** to sample and log high-frequency sensor data with minimal latency.

## ✨ Key Features

* ⚡ **Real-time interrupt-based sampling** every 20 ms
* 📥 **DMA-backed data access** for minimal CPU overhead
* 🔌 **Custom CAN-Bus handling** for decoding raw binary Correvit messages
* 🔧 **Clean, modular C++ architecture**
* 📊 **Structured SD card logging** with failure checks
* 🧩 **Supports analog, I²C, and CAN-based sensors**

---

## 🎯 What Is Measured

| Signal                         | Source         | Unit | Description                          |
| ------------------------------ | -------------- | ---- | ------------------------------------ |
| `Steering Angle`               | Potentiometer  | °    | Steering wheel position              |
| `Left/Right Wheel Angle`       | Potentiometer  | °    | Estimated wheel angles (via mapping) |
| `Acc X, Y, Z`                  | MPU6050 (I²C)  | g    | Linear acceleration                  |
| `Gyro X, Y, Z`                 | MPU6050 (I²C)  | °/s  | Angular velocity                     |
| `Vtot` (Total Velocity)        | Correvit (CAN) | km/h | Optical ground speed                 |
| `Vx, Vy`                       | Correvit (CAN) | km/h | Velocity components in x/y axes      |
| `Yaw Angle` (Raw & Calculated) | Correvit (CAN) | °    | Optical and computed yaw angles      |

---

## 🔌 Sensor Interfaces

### 🧭 IMU – *MPU6050* via I²C

* Measures 3-axis acceleration and gyroscopic rotation
* Calibrated on startup
* Accessed over I²C with precise bit parsing

### 🔄 Steering Potentiometer (Analog)

* Measures voltage proportional to steering angle
* Wheel angles are computed using calibration functions derived from a full vehicle axle alignment

### 📡 Correvit Ground-Speed Sensor (CAN Bus)

* High-end optical sensor for Vtot, Vx, Vy, yaw
* Data sent via 2 CAN messages (ID `0x01` and `0x02`)
* Custom DBC-style bit decoding and angle computation

---

## 💾 Data Logging

Logs are written to SD card in real time as tab-separated `.txt` files:

* `CAN_Trace.txt`: Groundspeed and yaw from Correvit
* `MPU_Trace.txt`: Steering and IMU data

Each file includes:

* Labeled columns
* Units
* Ready-to-import formatting for MATLAB, Python, or Excel

---

## 🧩 Architecture

* 🕒 **Interrupt Timer**: Fires every 20 ms for consistent sampling
* 📦 **Direct Memory Access (DMA)**: Optimized sensor reads and memory writes
* 📨 **CAN Listener**: Custom logic for Correvit decoding
* 🧱 **Modular Structs**: Separation between sensor logic and data handling
* 🖴 **Buffered Writes**: Data written in blocks to SD card to reduce wear

---

## 🧠 Requirements

* **Microcontroller**: Teensy 4.1
* **IMU**: MPU6050 via I²C
* **Steering Sensor**: Analog linear potentiometer
* **Ground Speed Sensor**: Correvit S-Motion or compatible
* **Storage**: Onboard SD card slot (Teensy 4.1)

---
