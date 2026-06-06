# ESP32 Main Firmware

Main firmware running on ESP32 for the AI Robotic Hand project.

## Responsibility
- Reads sEMG signals from MyoWare 2.0
- Runs TinyML CNN for autonomous slip detection (<50ms latency)
- Controls 6× MG946R servos via PCA9685 over I2C
- Sends haptic feedback commands via ESP-NOW

## Files
- `main.c` — Main application entry point and task scheduler

## Hardware Connections
| Pin | Connected To |
|-----|-------------|
| GPIO 21 (SDA) | PCA9685 SDA |
| GPIO 22 (SCL) | PCA9685 SCL |
| GPIO 34 (ADC) | MyoWare 2.0 OUTPUT |
| GPIO 5 | ESP-NOW Tx antenna |

## Status
🔄 In Progress
