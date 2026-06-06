# AI-Based Robotic Hand Control Using sEMG & Edge AI

> B.Tech Major Project · ECE @ JGEC · 2026  
> ESP32 · MyoWare 2.0 · MediaPipe · TinyML CNN · PCA9685 · ESP-NOW  
> **Total BOM: ₹14,604 — 34× cheaper than commercial alternatives (₹1–5 lakh)**

---

## 🧠 Project Overview

A full-stack AI-controlled robotic hand that combines Bayesian sensor 
fusion of two input modalities:
- **MediaPipe** hand-landmark vision — 30 FPS, 21 landmarks per frame
- **MyoWare 2.0 sEMG** forearm muscle signal acquisition

Fusion of both inputs reduces false positives by 60% vs single-sensor 
systems and enables robust occlusion recovery when vision is blocked.

---

## ⚡ Key Results

| Metric | Result |
|--------|--------|
| False positive reduction | 60% vs single-sensor baseline |
| Autonomous slip detection latency | <50 ms (TinyML CNN on ESP32) |
| Haptic feedback end-to-end latency | ~2 ms via ESP-NOW |
| Blindfolded object-density accuracy | >70% over 20 trials |
| Total BOM cost | ₹14,604 |
| Cost vs commercial alternatives | 34× cheaper (₹1–5 lakh range) |

---

## 🔧 System Architecture
MyoWare 2.0 sEMG ──┐
├──► Bayesian Fusion ──► TinyML CNN (ESP32)
MediaPipe Vision ──┘         │                      │
│                      ▼
│           PCA9685 12-bit PWM
│                      │
│              6× MG946R Servos
│           (InMoov 3D-printed hand)
│
└──► ESP-NOW ──► 5-finger Haptic Wristband
---

## 📁 Repo Structure
ai-robotic-hand-semg/
├── firmware/
│   ├── esp32_main/         ← Main ESP32 firmware (Embedded C)
│   ├── tinyml_model/       ← TinyML CNN (TensorFlow Lite for ESP32)
│   └── haptic_wristband/   ← ESP-NOW haptic feedback node
├── python/
│   ├── mediapipe_vision/   ← MediaPipe 21-landmark hand tracking
│   └── sensor_fusion/      ← Bayesian fusion of sEMG + vision
├── hardware/
│   ├── bom.md              ← Full Bill of Materials (₹14,604)
│   └── circuit_diagrams/   ← Wiring schematics
├── docs/
│   └── project_report.md   ← Full technical documentation
└── README.md
---

## 🛠️ Tech Stack

| Layer | Technology |
|-------|------------|
| Microcontroller | ESP32 |
| sEMG Sensor | MyoWare 2.0 |
| Vision | MediaPipe (30 FPS, 21 landmarks) |
| ML Model | TinyML CNN → TensorFlow Lite |
| Servo Driver | PCA9685 12-bit PWM |
| Actuators | 6× MG946R Servos |
| Force Sensing | FlexiForce sensors |
| IMU | MPU-6050 |
| Wireless | ESP-NOW (no router, ~2ms latency) |
| Hand Structure | InMoov open-source 3D-printed |
| Power | Custom buck-regulated 11.1V Li-Ion |
| Languages | Embedded C, Python |

---

## 💰 Bill of Materials

**Total: ₹14,604** — validated against commercial prosthetic/robotic 
hand alternatives priced at ₹1,00,000–₹5,00,000.  
Full component breakdown in [`hardware/bom.md`](./hardware/bom.md)

---

## 🏅 About

Built as B.Tech Major Project, JGEC ECE — demonstrating clinical-grade 
sEMG control, edge AI deployment, and full hardware-software co-design 
at a fraction of commercial cost.

---

## 📬 Contact

**Debjit Das** · B.Tech ECE @ JGEC  
📧 debjitdas.intern@gmail.com  
🔗 [LinkedIn](https://linkedin.com/in/debjit-das-00892b3a8)
