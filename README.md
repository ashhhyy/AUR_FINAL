# Autonomous Underwater Robot

Overview
This repository contains the code and resources for an Autonomous Underwater Robot using Raspberry Pi 4 and ESP32-CAM.

Features
- Raspberry Pi 4:
  - Motor control for 4 propellers (2 forward/backward, 2 up/down) using L298N drivers
  - Autonomous navigation logic with obstacle avoidance and stabilization
  - Ultrasonic sensors (front, back, bottom) for obstacle detection and depth measurement
  - Flask web app for control and dashboard

- ESP32-CAM:
  - Captures images every 10 seconds
  - Saves images to SD card

- Web Dashboard:
  - Start/stop motion control
  - View 5 latest images captured by ESP32-CAM

Hardware Specifications
- Raspberry Pi 4 Model B (4GB)
- 4x Jet Boat Underwater 3-blades Propeller Motor Engines (2 for forward/backward, 2 for up/down)
- 3x AJ-SR04M Waterproof Ultrasonic Sensors (front, back, bottom)
- ESP32-CAM with OV2640 camera and microSD card
- L298N Motor Driver Dual H-Bridge
- LM2596S DC-DC Buck Converter
- Lithium Battery 11.1V 2000mAh

Directory Structure
- `rpi/` - Raspberry Pi Python code for motor control, sensors, autonomous logic, and Flask app
- `esp32-cam/` - Arduino sketch for ESP32-CAM image capture and SD card saving
- `web-dashboard/` - Web dashboard for control and image viewing

