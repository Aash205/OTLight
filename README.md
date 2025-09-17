# 🩺 Surgery Recording Camera Unit

A **compact, high-resolution Full HD camera system** designed for seamless integration into any **Operation Theatre (OT) light setup**.
It enables **live transmission & recording of surgeries** for:

* ✅ Quality control
* 🎓 Education
* 🔬 Research

Supports **advanced surgical techniques** like *laparoscopic* and *robot-assisted procedures*.

---

## 📷 Device Preview

![Image](https://github.com/user-attachments/assets/bbd29b44-30cb-45c5-bb2b-5d922355f859)

---

## ⚡ Technical Overview

| Feature             | Specification                                       |
| ------------------- | --------------------------------------------------- |
| **Microcontroller** | STM32 series (ARM Cortex-M)                         |
| **Video**           | Full HD 1920×1080 @ 30fps, Autofocus                |
| **Lighting Range**  | Reliable up to 100,000 Lux                          |
| **Connectivity**    | Wi-Fi module (streaming & control)                  |
| **Battery**         | Rechargeable Li-ion, \~6 hours backup               |
| **Battery Mgmt.**   | Charging control, Voltage monitoring, LED indicator |
| **Mounting**        | Universal OT light head adapter                     |
| **Sterilization**   | Autoclavable cover for infection control            |

---

## 🛠️ Running the Code on STM32

### 1️⃣ Setup Development Environment

* Install **STM32CubeIDE** or preferred STM32 toolchain
* Configure project for your STM32 model (e.g., `STM32F4xx`)

### 2️⃣ Configure Peripherals

* Initialize **Camera Interface** → DCMI / SPI / UART
* Setup **Wi-Fi module** → UART / SPI
* Configure **Battery Monitoring** → ADC / GPIO
* Initialize **Timers** → FPS control + Autofocus handling

### 3️⃣ Flash the Firmware

```bash
# Build project
make all

# Flash using ST-Link
st-flash write firmware.bin 0x8000000
```

### 4️⃣ Battery Management

* Read battery voltage via **ADC**
* Control charging/discharging with **GPIO**
* Drive LED indicator → show battery % thresholds

### 5️⃣ Run & Test

* 🔌 Power ON device → verify **camera initialization**
* 📡 Connect to Wi-Fi → check streaming software
* 🔋 Check **battery LED indicator**
* 🎥 Start **video streaming/recording**

---
