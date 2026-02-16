# Speed Monitoring System (EECS 2032)

## Overview
This project implements a Speed Monitoring System using the KL43Z microcontroller.  
The system measures the speed of vehicles passing through a sensor setup, compares it with a configurable speed limit, and calculates fines based on the measured speed. LEDs indicate whether the vehicle is within the limit or fined.
![Board Setup](docs/Photos/complete_setup.JPG)

This project was completed as part of EECS 2032 (Embedded Systems) at York University.

---

## Features
- Measures vehicle speed using two sensors with timing constraints.
- Configurable speed limit via an analog input (potentiometer).
- Calculates fines based on speed relative to the limit.
- LED indicators to show speed violations.
- Bash script included to process log files from testing sessions.
- Modular and maintainable C code, using embedded system best practices.

---

## File Structure
```
Lab8_Final/
├─ src/
│ └─ Mini_Project_Lab8_C_code.c
├─ scripts/
│ └─ Mini_Project_Lab8_Bash_Script.sh
├─ docs/
│ ├─ photos/
│ │ ├─ KL43Z_board_setup.JPG
│ │ ├─ sensor_wiring.JPG
│ │ └─ ... (rest of photos)
│ └─ videos/
│ └─ Lab8_Video1.mp4
└─ README.md
```

---

## Hardware Setup
- Microcontroller: KL43Z board
- Sensors: Two distance sensors for vehicle detection
- Actuators: LEDs for visual indication of fines
- Connections: Wiring between sensors, LEDs, and microcontroller follows the lab instructions (see photos in `docs/photos/`)

---

## Bash Script
The included bash script `Mini_Project_Lab8_Bash_Script.sh`:
- Processes log files from vehicle testing (`putty.log`)  
- Extracts vehicle ID and speed data  
- Produces `final_output.txt` summarizing test results  

> Note: Bash script requires a `.log` file to run.  

---

## Photos & Video
- Photos of the physical hardware: `docs/photos/`
- Demo video of the project: `docs/videos/Lab8_Video1.mp4`  

---

## Skills Demonstrated
- Embedded Systems Programming: C programming for KL43Z microcontroller  
- Hardware Integration: Sensor and LED interfacing  
- Timing & Real-Time Constraints: Vehicle speed measurement under strict timing  
- Scripting & Automation: Bash script for log processing  
- Collaboration: Pair project, structured lab workflow  
- Documentation: Screenshots and project overview for reproducibility  

