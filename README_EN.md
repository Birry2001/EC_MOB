

# LIMO-DAVIS: Event-Based Dynamic Obstacle Detection for Robust Navigation

## General Overview

This repository contains a **Master’s level project in robotics and artificial perception**, whose goal is to **improve the robustness of autonomous navigation on the AgileX LIMO robot** under challenging conditions by integrating a **DAVIS-346 event-based camera** into the existing ROS 2 stack.

The project follows an **experimental and comparative approach**, first evaluating the performance of the existing SLAM + Navigation pipelines on the LIMO platform, and then proposing an **event-based dynamic obstacle detection module** integrated into **Nav2**.

---

## Project Objectives

### Main Objective

Improve the overall **SLAM + Navigation** performance of the LIMO robot in degraded conditions (low light, HDR, fast motions, dynamic obstacles) by exploiting the capabilities of an event-based camera.

### Specific Objectives

* Quantitatively evaluate the existing navigation pipelines:

  * **Cartographer LiDAR + Nav2**
  * **RTAB-Map RGB-D + Nav2**
* Implement an **event-based dynamic obstacle detection method**
* Integrate this detection into the **Nav2 costmap**
* Compare navigation performance **with and without the DAVIS camera** under identical scenarios

---

## Robotic Platform

* **Robot**: AgileX **LIMO ROS 2**
* **Operating System**: Ubuntu 22.04
* **Middleware**: ROS 2 Humble
* **Onboard Computing**: Intel NUC i7

### Sensors Used

* **2D LiDAR**: EAI T-mini Pro
* **RGB-D Camera**: Orbbec Dabai
* **IMU**: LIMO onboard IMU
* **Event-based Camera**: **DAVIS-346** (events + APS + IMU)

---

## Scientific Approach

### Selected Event-Based Method

The main dynamic perception module is based on the following paper:

> **Zhao, Li, Lyu – “Event-based Real-time Moving Object Detection Based on IMU Ego-motion Compensation”**,
> *IEEE ICRA 2023*

Key principles:

1. **Ego-motion compensation** using IMU measurements
   → nonlinear warping of event streams
2. **Dynamic segmentation** based on *time images* and *count images*
3. **Clustering of moving objects** (DBSCAN + motion information)
4. **Projection of dynamic objects** into a Nav2 costmap layer

---

## Current Repository Organization



---

## 📚 Main References

* Zhao et al., *Event-based Real-time Moving Object Detection Based on IMU Ego-motion Compensation*, ICRA 2023
* RTAB-Map ROS
* Cartographer ROS
* Nav2 Documentation
* AgileX LIMO ROS 2 User Manual

---

## 👤 Authors

**Nochi Magouo**
MSc (Master 2) in Robotics – Artificial Perception
Université Clermont Auvergne / Institut Pascal

**Nadjib Mekelleche**
MSc (Master 2) in Robotics – Artificial Perception
Université Clermont Auvergne / Institut Pascal


## License

This project is licensed under the MIT License – see the [LICENSE.md](LICENSE.md) file for details.
