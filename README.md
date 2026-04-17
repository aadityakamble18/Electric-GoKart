# 🏎️ Electric Go-Kart: Custom Traction Control & BMS on a PVC Chassis

![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)
![Platform: Arduino](https://img.shields.io/badge/Platform-Arduino-00979C.svg)
![Simulation: MATLAB](https://img.shields.io/badge/Simulation-MATLAB-e16723.svg)

👋 **Hi there! We are Aaditya Amresh Kamble and Yash Kumar.** We are final-year Materials Engineering students at the **Indian Institute of Technology Jodhpur**, working under the guidance of **Dr. Srijan Sengupta**. 

This repository contains the complete source code, MATLAB simulations, and hardware schematics for our capstone project: a fully functional, low-cost electric go-kart built from scratch. We wanted to step out of our materials engineering comfort zone and tackle a complex, multi-domain problem combining mechanical design, embedded C++ programming, and control theory. 

The result? An EV platform that actively manages its own wheel slip and battery safety—all running on a $4 microcontroller!

---

## ✨ Key Features

1. **PVC Ladder-Frame Chassis:** We validated the use of solvent-welded Schedule-40 PVC for a low-cost, structurally sound EV prototype frame (with steel inserts at high-stress nodes).
2. **Embedded Traction Control System (TCS):** Because we used dual rear BLDC hub motors without a mechanical differential, we built a proportional feedback TCS in software. It catches and corrects wheel slip in under **240 ms**, preventing spin-outs and reducing current spikes by over 57%.
3. **Custom VRLA Battery Management System (BMS):** A rigid safety watchdog for our 36V Lead-Acid pack. It enforces strict cutoffs for undervoltage (33V), overcurrent (45A), and overtemperature (55°C) to prevent irreversible plate sulfation and thermal runaway.
4. **MATLAB Digital Twin:** A complete mathematical model of the kart's motor dynamics and battery degradation (incorporating Peukert's Law) with custom GUI dashboards to safely tune control gains before track testing.

---

## 📂 Repository Structure

```text
📦 Electric-GoKart
 ┣ 📂 Arduino_Code
 ┃ ┣ 📜 Main_Kart_Logic.ino       # Core code containing TCS and BMS loops
 ┃ ┣ 📜 Sensors_Config.h          # INA226 and DS18B20 setup
 ┃ ┗ 📜 StateMachine.cpp          # Precharge and fault-handling logic
 ┣ 📂 MATLAB_Simulations
 ┃ ┣ 📜 EV_Dynamics_Model.m       # Euler integration script for motor physics
 ┃ ┣ 📜 TCS_DigitalTwin.mlapp     # Interactive GUI for Traction Control
 ┃ ┗ 📜 BMS_Visualizer.mlapp      # Interactive GUI for Battery Management
 ┣ 📂 Hardware_Schematics
 ┃ ┣ 📜 Electrical_Architecture.pdf
 ┃ ┣ 📜 PVC_Frame_CAD.SLDPRT
 ┃ ┗ 📜 Steering_Ackermann_2D.pdf
 ┣ 📜 Research_Paper.pdf          # Full IEEE-formatted project paper
 ┗ 📜 README.md

## 🛠️ Hardware & Components Overview

* **Microcontroller:** Arduino Nano (ATmega328)
* **Motors:** 2x 36V 350W Brushless DC (BLDC) Hub Motors with Hall Sensors
* **Controllers:** 2x standard 36V/50A E-bike motor controllers (modified via RC low-pass filters to accept PWM from the Arduino)
* **Battery:** 4x 12V 7.2Ah VRLA batteries wired in series (36V nominal)
* **Sensors:** INA226 (I2C Current/Voltage Monitor), DS18B20 (1-Wire Temperature Probes)
* **Safety Hardware:** 200A main contactor, 47Ω/25W precharge resistor, 100A ANL fuse, emergency kill-switch.

---

## 🚀 How to Run the Simulations

We highly recommend playing with the MATLAB simulations before touching the Arduino code. It helps to understand the physics of what the kart is actually doing.

1. Clone this repository: `git clone https://github.com/aadityakamble18/Electric-GoKart.git`
2. Open MATLAB (R2021a or newer recommended).
3. Navigate to the `MATLAB_Simulations` folder.
4. Run `TCS_DigitalTwin.mlapp` to launch the Traction Control dashboard.
5. Use the sliders to apply throttle, and hit the **"Induce Slip"** button to watch the algorithm catch and correct the wheel speed difference in real-time!

---

## 💻 Uploading to the Kart

1. Open the `Arduino_Code` folder using the Arduino IDE.
2. Ensure you have the required libraries installed via the Library Manager:
   * `Wire.h` (Built-in)
   * `DallasTemperature` (For DS18B20)
   * `OneWire`
   * `INA226_WE` (Or your preferred INA226 library)
3. Connect your Arduino Nano, select the correct COM port, and hit Upload.
4. **Safety Warning:** *Always* test the logic with the drive wheels lifted off the ground first!

---

## 📈 Real-World Performance

We extensively field-tested this kart. Our data logged an acceleration of 0-15 km/h in ~4 seconds (with a 75kg driver). When we simulated an ice patch by lubricating one rear tire, the Arduino detected the free-spinning wheel and clamped the PWM output down, maintaining straight-line stability just as our MATLAB model predicted. Over 45 minutes of hard track testing, our BMS threw zero nuisance trips.

---

## 📖 Read Our Paper

For a deep dive into the math, physics, and control algorithms, please read our full IEEE-formatted research paper included in this repository: `Research_Paper.pdf`.

---

## 🙏 Acknowledgments

A massive thank you to our guide, **Dr. Srijan Sengupta**, for his patience, advice on dealing with ground-loop issues, and for encouraging us to push the boundaries of what materials engineering students can build.

*Built with sweat, late nights, and a lot of PVC primer at IIT Jodhpur.* 🇮🇳
