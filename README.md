# VitalsPatch

**A Wearable Multi-Patient Health Monitoring System**
*EE 475 — Embedded Systems Capstone, University of Washington*

![Status](https://img.shields.io/badge/status-Phase%202%20%2F%20In%20Development-yellow)
![Platform](https://img.shields.io/badge/MCU-STM32F4%20%7C%20ESP32--C3-blue)
![UI](https://img.shields.io/badge/dashboard-Raspberry%20Pi%204-red)

## Overview

VitalsPatch is a wearable health monitoring system built for clinical and care environments where one practitioner needs to watch over multiple patients at once. The device continuously tracks **heart rate, SpO2, body temperature, and motion**, and detects **falls and out-of-range vitals** in real time — consolidating what normally takes several single-purpose wearables into one device and one dashboard.

## System Architecture

| Stage | Component | Role |
|---|---|---|
| Sensing | MAX30102 | Heart rate & SpO2 (optical PPG) |
| Sensing | MPU-6050 | Motion & fall detection (accel + gyro) |
| Sensing | Temperature sensor | Skin/body temperature |
| Acquisition | ESP32-C3 | Sensor-side acquisition & wireless packet transmission |
| Processing | STM32F4 | Packet validation, threshold analysis, alert codes |
| Interface | Raspberry Pi 4 | Multi-patient dashboard, trend graphs, logging |
| Power | LiPo battery | Regulated 3.3V rail |

## Alert Codes

| Code | Event |
|---|---|
| 0 | Normal — all readings within range |
| 1 / 2 | Heart rate above / below threshold |
| 3 / 4 | Temperature spike / drop |
| 5 | Low SpO2 |
| 6 | Fall detected (accelerometer/gyroscope) |

## Development Progress

**Phase 1 — Independent Modules** ✅
- Sensor module reading and packetizing all required signals
- STM32 generating correct alert codes for every event type
- Dashboard rendering near real-time graphs and prioritized alerts on mock data
- Functional sensor → STM32 communication

**Phase 2 — Full Pipeline & Multi-Patient Support** 🔄
- ✅ Two-patient dashboard with clear data separation, prioritized alerts, per-patient logs
- ✅ Two independent sensor modules transmitting with patient ID
- 🔄 Finalizing wired serial link from STM32 → Raspberry Pi

## Testing & Validation

- **Sensors:** MAX30102 cross-checked against an Apple Watch; MPU-6050 validated against standing, walking, and simulated falls; temperature checked against an external thermometer; motion-artifact rejection confirmed via deliberate wrist movement during sampling.
- **Alert logic:** Full 9-case test suite (baseline, each alert individually, combined multi-alert scenarios) — all passing.
- **Dashboard:** Validated via console-replayed test packets and concurrent two-patient simulation, confirming correct visual prioritization and patient data separation.
- **End-to-end:** Live BLE → STM32 → serial → Raspberry Pi pipeline testing in progress; preliminary latency within acceptable range for clinical use.

## Roadmap

**This term:** full serial integration, wearable packaging (breadboard → patch form factor), refined fall-detection thresholds, multi-patient stress testing
**Post-term:** per-patient calibration profiles, ML-based predictive hypoxia alerting, structured data export/logging
**Long-term:** custom PCB, NSF SBIR Phase I application, scaling to 4+ simultaneous patients

## Team

| Member | Role |
|---|---|
| Eeshani Shilamkar | UI design & project management |
| Arnav Mohan | Sensor integration & wireless communication |
| Maya Desai | Data processing & trend analysis (STM32) |
| Nate Snyder | Sensor calibration & wearable assembly |
| Anushka Misra** | Hardware integration & testing; sensor-side acquisition firmware |
| Bobby Taing | Alert logic development & system testing |
