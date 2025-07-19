# 🤖 RCJE-Team-ResQ

This repository features a **scalable, competition-optimized robot controller** developed for the **RoboCup Junior Rescue Simulation Maze**, fully compatible with **Webots** and the **Erebus framework**.

Built in **Python**, the system supports advanced functionalities including **victim identification**, **hazard detection**, **checkpoint tracking**, and **automated recovery from entrapment**.
Our solution secured a **Top 5 position** in the **RoboCup Junior Egypt Rescue Simulation Maze**.

---

## 📚 Table of Contents

* [Overview](#-overview)
* [Sample Score Display](#-sample-score-display)
* [Features](#-features)
* [Quick Setup](#-quick-setup)
* [Tech Stack](#-tech-stack)
* [Architecture](#-architecture)
* [How It Works](#-how-it-works)
* [Team Members](#-team-members)
* [Personal Achievement](#-personal-achievement)
* [Contributions](#-contributions)
* [Contact & Questions](#-contact--questions)

---

## 🌐 Overview

This intelligent controller leverages **Webots** and **Erebus** to simulate realistic rescue scenarios in dynamic environments. Its **modular structure** facilitates rapid development, testing, and debugging of autonomous robot behavior.

---

## 📊 Sample Score Display
<img width="2552" height="1400" alt="image" src="https://github.com/user-attachments/assets/99f666de-7a37-4cca-aa69-2871b56ffc78" />

**Score Output (Webots UI)**

---

## 🎥 Robot Demo
![Uploading Sample.gif…]()


---

## 🚀 Features

### 🗺️ Navigation & Path Planning

* **Obstacle Avoidance:** Utilizes LIDAR for real-time environmental scanning and route adjustment.
* **Checkpoint Recognition:** Records start zones and waypoints to support re-entry and scoring logic.
* **Stuck Recovery:** Performs adaptive 180° maneuvers upon detecting entrapment.

### ⚠️ Hazard Identification

* **Surface Hazards:** Differentiates black tiles and swamp zones using calibrated color sensors.
* **Chemical Threats:** Detects specific hazardous materials (e.g., poison, flammable gas) using pattern recognition.

### 🧍 Victim Detection

* **Camera Analysis (OpenCV):** Processes visual input to locate H, S, and U victims.
* **Color Filtering:** Applies HSV-based segmentation to ensure robustness under varying light conditions.

### 🚶 Motion Control

* **Wall-Following:** Default navigation logic optimized for narrow corridors.
* **Dynamic Path Correction:** Adapts trajectory upon detecting floating walls using LIDAR and checkpoints.

---

## 🛠️ Quick Setup

### ✅ Prerequisites

* **Webots R2023b**
* **Erebus Framework v24.0.0**
* **Python 3.10+**

### ⚙️ Installation Steps

```bash
git clone https://github.com/your-username/RCJE-Team-ResQ.git  
cd RCJE-Team-ResQ
pip install opencv-python numpy
```

1. Open the project in **Webots R2023b**
2. Launch the simulation environment and run the controller

---

## 🧰 Tech Stack

| Component | Version | Description                   |
| --------- | ------- | ----------------------------- |
| Webots    | 2023b   | Robot simulation environment  |
| Erebus    | V24.0.0 | RCJ simulation framework      |
| Python    | 3.10    | Primary development language  |
| OpenCV    | 4.x     | Computer vision processing    |
| NumPy     | 1.23+   | Numerical computation toolkit |

---

## 🧠 Architecture

```
controller.py
├── Robot Initialization
│   ├── Sensor & Motor Setup (LIDAR, Camera, GPS, Motors)
│   └── Movement Constraints (Speed, Turn Radius)
├── Navigation & Mapping
│   ├── Position Tracking (GPS + Compass)
│   └── Heading Adjustment (Dynamic Correction)
├── Detection Modules
│   ├── Hazard Recognition (Color Sensors)
│   └── Victim Detection (OpenCV + Camera)
├── Behavior Logic
│   ├── Wall-Following
│   └── Recovery Maneuvers
└── Main Loop
    ├── Timed Updates
    └── State Persistence (Victim Logs, Hazard History)
```

---

## 🔍 How It Works

### 🧭 Navigation Algorithm

* **Left-Wall Following:** Maintains optimal clearance from walls using LIDAR to ensure reliable maze traversal.

### 👁️ Visual Processing

#### Victim Detection

* Real-time analysis of camera frames using **OpenCV**
* Identifies H, S, U letters via **HSV color filtering**
* Logs GPS coordinates of confirmed victims

#### Hazard Detection

* Recognizes black tiles, swamp zones, and chemical labels
* Uses **template matching** and color classification for accuracy

### 🔄 Stuck Recovery Protocol

* Generates virtual checkpoints every 10 seconds
* Triggers a recovery routine (reverse and turn) when the same checkpoint is crossed repeatedly in a short timeframe

---

## 👥 Team Members

**Khaled Mohamed**
**Team Lead & Solo Developer**

* Architected the entire control system
* Implemented navigation and obstacle detection algorithms
* Integrated all sensors (LIDAR, camera, GPS, color sensors)
* Designed the visual detection pipelines
* Tuned system performance for competition execution

---

## 🏆 Personal Achievement

This project was **fully self-developed** and led to a **Top 5 national ranking** in the RoboCup Junior Egypt Rescue Simulation Maze.

It demonstrates:

* Complete control system design expertise
* Strong robotics and AI implementation skills
* Real-time problem-solving in constrained, dynamic environments

---

## 🤝 Contributions

We welcome community involvement! You can help by:

### 🐛 Reporting Bugs

Please include:

* Clear issue title (e.g., “LIDAR misreads in curved corridors”)
* Webots & Python version
* Reproduction steps and expected behavior
* Screenshots, logs, and error messages

### 💡 Suggesting Features

Before opening a new request:

* Check open issues for duplicates
* Include:

  * Prefix "Feature Request:" in the title
  * Detailed use-case description
  * Optional technical approach or mockup

---

## 📬 Contact & Questions

For inquiries, collaboration, or feedback, feel free to reach out:

* **Email:** [khaledabdulla@gmail.com](mailto:khaledabdulla@gmail.com)
* **GitHub:** [github.com/khaled1234kh](https://github.com/khaled1234kh)
* **LinkedIn:** [linkedin.com/in/khaled-mohamed-22a22a325](https://linkedin.com/in/khaled-mohamed-22a22a325)
