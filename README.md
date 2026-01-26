# 🤖 Bluetooth-Controlled-Robot-with-ADC-I2C-FreeRTOS 

### Real-Time Embedded System for Multi-Motor Control & Sensor Acquisition

📍 **Domain**: Embedded Systems · Robotics · Real-Time OS
🧠 **Technologies**: STM32F4 · FreeRTOS · C · PWM · ADC · UART · I2C

---

## 🚀 Project Summary

This project is a **real-time robotic control system** developed on the **STM32F4 Discovery (STM32F407VG)** platform.
It demonstrates **professional embedded software practices**, including **multitasking with FreeRTOS**, **direct peripheral control**, and **robust inter-task communication**.

The system controls **four DC motors**, acquires **multi-channel analog data**, measures **temperature via I2C**, and supports **remote control and telemetry via UART**.

👉 Designed as a **portfolio-level embedded project** showcasing **RTOS, low-level drivers, and system architecture skills**.

---

## 💡 What This Project Demonstrates

✔ Real-Time multitasking with **FreeRTOS**
✔ Low-level **STM32 peripheral programming (no HAL)**
✔ Deterministic motor control using **PWM timers**
✔ Interrupt-driven **ADC acquisition**
✔ **UART protocol design** (command + telemetry)
✔ Clean **task-based architecture**
✔ Embedded system debugging & timing analysis

---

## 🧩 System Capabilities

### 🕹️ Motor Control

* 4 DC motors (independent control)
* PWM via **TIM4**
* Direction management (Forward / Backward / Left / Right / Stop)
* Emergency stop with highest RTOS priority

### 📊 Sensor Acquisition

* 3 ADC channels (12-bit resolution)
* Timer-triggered conversions (TIM2)
* Scan mode + interrupt handling
* Thread-safe data transfer via queues

### 🌡️ Temperature Monitoring

* DS1621 digital temperature sensor
* I2C communication (100 kHz)
* Periodic temperature reporting

### 📡 Communication

* UART (USART2, 9600 baud)
* Text-based command interface
* Automatic telemetry transmission
* Smartphone / PC compatible (Bluetooth or USB-UART)

---

## ⚙️ Software Architecture (FreeRTOS)

| Task             | Priority | Responsibility          |
| ---------------- | -------- | ----------------------- |
| ADC Tasks (x3)   | Low      | Analog data acquisition |
| Data Aggregation | Medium   | Sensor fusion           |
| UART Telemetry   | Medium   | Data transmission       |
| Command Handler  | High     | Robot control           |
| Emergency Stop   | Highest  | Safety mechanism        |

🧠 **Queues & Semaphores** ensure safe inter-task communication
⏱️ **Timers & ISRs** guarantee deterministic behavior

---

## 🧪 Peripherals Used

| Peripheral | Usage                     |
| ---------- | ------------------------- |
| TIM4       | PWM motor control         |
| TIM2       | ADC triggering            |
| ADC1       | Multi-channel acquisition |
| USART2     | Communication             |
| I2C1       | Temperature sensor        |
| EXTI       | External interrupt        |

---

## 🛠️ Tech Stack

* **Language**: C (Embedded)
* **RTOS**: FreeRTOS
* **MCU**: STM32F407VG
* **IDE**: Keil µVision
* **Drivers**: STM32 Standard Peripheral Library
* **Debug**: ST-Link

---




