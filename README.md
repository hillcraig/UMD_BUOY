# UMD Buoy

Welcome to the University of Minnesota Duluth (UMD) open‑source environmental‑monitoring buoy platform. This repository bundles two buoy firmware stacks and a full IMU wave‑processing library so researchers can go from raw sensor data to science‑grade wave spectra on low‑power hardware.

---

## Repository Layout

```text
.
├── LICENSE
├── LTE_Buoy/                 # Cellular buoy (Blues Swan R5 + Notecard LTE)
│   └── LTE_Buoy.ino
|   └── platformio.ini
├── SAT_Buoy/                 # Iridium buoy 
│   └── Iridium_Buoy_Final/Iridium_Buoy_Final.ino
├── wave_processing/          # Real‑time wave‑spectrum stack
│   ├── python_processing/    # Exploratory notebooks & data
│   └── wave_processor/       # Tiny C library (+ kissFFT) for MCU deployment
└── README.md
```

---

## Overview

| Sub-project          | Purpose                                                                                                                                                                                                                                   |
|----------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| **LTE_Buoy**         | Production firmware built on **stm32duino**. Uses the Blues **Swan R5** MCU and the **Notecard** for LTE-M/NB-IoT backhaul.                                                                                                               |
| **SAT_Buoy**         | Arduino-based Iridium Short Burst Data buoy. ***Placeholder – details coming soon.***                                                                                                               |
| **wave_processing**  | Tools for turning 3-axis IMU acceleration into wave energy spectra.<br>• **wave_processor** – C code for the buoy MCU.<br>• **python_processing** – Jupyter notebooks for algorithm R&D. |

---

## Getting Started

1. **Clone the repository**

   ```bash
   git clone https://github.com/hillcraig/UMD_BUOY.git
   cd UMD_BUOY
   ```

2. **Open the desired project** in your IDE (we recommend **PlatformIO** for its automatic library management):  
   * `LTE_Buoy/LTE_Buoy.ino` – cellular buoy (requires Notehub setup)  
   * `SAT_Buoy/Iridium_Buoy_Final/Iridium_Buoy_Final.ino` – Iridium buoy  

3. **Install dependencies**  
   * **PlatformIO:** libraries are auto-installed from `platformio.ini`.  
   * **Arduino IDE:** manually install the libraries listed in `platformio.ini`.  

4. **Add the wave-processing library**  
   * Copy `wave_processing/wave_processor` into the `lib/` folder of your PlatformIO project, **or** place it in your Arduino `libraries` directory and restart the IDE.  

5. **(LTE_Buoy only)** Create a Notehub ProductUID – sign up at <https://notehub.io>, create a project, copy the ProductUID, and update the `productUID` constant in `LTE_Buoy.ino`.  

6. **Flash** to a **Blues Swan R5** (LTE) or your chosen **SAMD/STM32** target (Iridium).  

7. Verify a GPS fix, sensor readings, and network connectivity before field deployment.

---

## Deployment Checklist

* ✅ **Firmware flashed** (latest `main` build).
* ✅ **Battery/Solar** connected & charging.
* ✅ **SD card prepped** (see below).
* ✅ **Sensors** reporting nominal values.
* ✅ **Water‑tight enclosure** sealed.

### SD‑Card Preparation & File Naming

1. Format as **FAT32**.
2. Create a single file named `BUOYYYYY.txt` where `YYYY` is the current year (e.g. `BUOY2025.txt`).
3. Insert this header line as the first line:

   ```csv
   yyyy,MM,dd,hh,mm,ss,sats,lat,lon,airTemp,waterTemp,conductivity,dissolvedOxygen,pH,orp,sinr,rssi,pressure,voltage,wave_height,wave_period
   ```

> **Note:** File names follow the 8.3 convention (max 8‑character base + 3‑character extension).

---

## Collected Data – Cloud & Dashboards

Live data can be visualized through:

- **Notehub** – Blues Wireless service that buffers sensor “notes” from the Notecard and exposes them via REST/webhooks.  
  <https://notehub.io>

- **Datacake** – low-code dashboard platform we use for charts, maps, and alerts. Buoy data is forwarded from Notehub via webhook.  
  <https://www.datacake.co/>

See the complete setup tutorial (Notehub project, webhooks, and dashboard import):  
<https://github.com/hillcraig/UMD_BUOY/blob/main/docs/cloud_setup.md>

---

## Hardware Bill of Materials (core build)

| Subsystem                | Part(s)                                                        |
|--------------------------|----------------------------------------------------------------|
| MCU & Cellular           | Blues **Swan R5** MCU + **Notecard** (LTE-M / NB-IoT)          |
| IMU                      | **ICM-20948** 9-DoF                                            |
| Air Temperature          | **TSYS01**                                                     |
| Water Temperature        | Atlas Scientific **EZO-RTD**                                   |
| Electrical Conductivity  | Atlas Scientific **EZO-EC**                                    |
| Dissolved Oxygen         | Atlas Scientific **EZO-DO**                                    |
| pH                       | Atlas Scientific **EZO-pH**                                    |
| ORP                      | Atlas Scientific **EZO-ORP**                                   |
| Water Pressure (optional)| **MS5803-14BA** (SparkFun) — currently commented out in code   |
| Storage                  | Micro SD (8.3 filenames) via **SparkFun Qwiic I²C Mux**        |
| I²C Multiplexer          | **SparkFun Qwiic Mux** (TCA9548A)                              |
| Status LEDs              | Surface blue LED on A4; optional underwater illumination LED   |


---

## Wave‑Processing Research Notes

Partial implementlementatin of the vertical‑acceleration correction and FFT segmentation strategy outlined in **Bender et al., 2010**. See `wave_processing/python_processing/Bender_et_al.ipynb` for derivations and validation plots.


---

## Known Issues

Open problems:

1. LTE_Buoy: Pressure sensor causes system issues.
2. LTE_Buoy: I2C error after querying notecard for voltage.
3. LTE_Buoy: incomplete implementation of wave processing methods from Bender et al, 2010


---

## Roadmap / Vision

* **Zephyr RTOS** – enable continuous IMU sampling in parallel with other tasks (see *Note-Zephyr*).
* **Completed / Better wave-processing** – complete Bender et al. implementation **or** adopt a superior approach.  
  Eventually add **directional** wave processing.
* **Unit Testing for wave processing** - add unit testing to wave processing methods in both C and python to validate output with labled data. 
* **Custom Sensor PCB** – consolidate all sensor and interface components onto a single integration PCB.
* **Open-source Hull CAD** – publish 3-D-printable CAD files, bill of materials, and assembly docs for the buoy hull and enclosure.


---

### Contibuting Questions 

For questions and contributing, feel free to reach out:

- Liam Gaeuman: liamgaeuman [at] gmail [dot] com  
  → Main developer responsible for all software, firmware, and system integration.
- Craig Hill: cshill [at] d [dot] umn [dot] edu  
  → Faculty advisor and project lead overseeing hardware and deployment strategy.

---

© UMN Duluth, 2025 – Licensed under the MIT License.
