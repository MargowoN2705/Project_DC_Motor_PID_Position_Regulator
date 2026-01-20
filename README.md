# DC Motor Cascaded PID Position Controller (STM32)

This project implements a **Cascaded PID Controller** for a DC Motor using an **STM32L476RG** microcontroller. The system operates in a "Master-Slave" configuration, where the motor tracks the position of a manual reference encoder (handwheel).

The project includes hardware firmware (C/HAL) and a simulation model (Python) for control strategy validation.

## 🚀 Key Features

* **Cascaded Control Architecture:**
    * **Outer Loop:** P-Controller for Position.
    * **Inner Loop:** PID Controller for Angular Velocity.
* **Real-Time Execution:** Deterministic 500 Hz control loop triggered by hardware timer interrupts.
* **Signal Processing:** 2nd-order IIR Butterworth filter applied to the velocity feedback to suppress quantization noise.
* **Robustness:**
    * **Anti-Windup:** Clamping method to prevent integral saturation.
    * **Output Saturation:** Voltage limiting (±3.0V) and PWM Deadzone handling.
* **Telemetry:** Real-time data logging via UART (115200 baud) for tuning and visualization.

## 🛠 Hardware Configuration

* **MCU:** STM32L476RGTx (Nucleo-L476RG Board)
* **Motor Driver:** H-Bridge (PWM + Direction pins)
* **Sensors:** Two Quadrature Encoders (Master Reference & Motor Feedback)

### Pinout Mapping

| STM32 Pin | Function | Peripheral | Description |
| :--- | :--- | :--- | :--- |
| **PA6, PA7** | Encoder A/B | `TIM3` | **Motor Feedback Encoder** (Slave) |
| **PC6, PC7** | Encoder A/B | `TIM8` | **Reference Input Encoder** (Master/Knob) |
| **PB6** | PWM Output | `TIM4_CH1` | PWM Signal for Motor Driver |
| **PB4** | GPIO Output | `GPIO` | Motor Direction IN1 |
| **PB5** | GPIO Output | `GPIO` | Motor Direction IN2 |
| **PA2, PA3** | UART TX/RX | `USART2` | Debugging & Telemetry PC Interface |
| **PA5** | LED | `GPIO` | Status LED |

*Block Diagram: Cascaded Control Structure*

## 🧠 Control Algorithm

The system treats the DC Motor as a second-order inertial object. The control law is calculated inside the `HAL_TIM_PeriodElapsedCallback` (Timer 2 Interrupt) every **2 ms**.

### 1. Position Loop (Outer)
Calculates the desired velocity based on position error.
$$\omega_{target} = K_{p\_pos} \cdot (\theta_{ref} - \theta_{meas})$$
*Saturated at ±12.0 rad/s.*

### 2. Velocity Loop (Inner)
Regulates the voltage applied to the motor to track the target velocity.
$$u(t) = K_p e(t) + K_i \int e(t) dt + K_d \frac{d}{dt}e(t)$$
*Includes IIR filtering on feedback and anti-windup clamping on the integrator.*

## 💻 Software Structure

### Firmware (`/Core/Src`)
* `main.c`: Contains the main loop and the Control Loop ISR.
    * **Sampling Time ($T_s$):** 0.002s (500 Hz)
    * **Filter:** 2nd order Low-pass Butterworth (Cutoff: 50Hz)
* `tim.c`: Hardware timer configurations (Encoder mode, PWM generation, Interrupt base).

### Simulation (`/Simulation`)
* `Projekt Symulacja.py`: A Python script using `numpy` and `scipy`.
    * Simulates the motor plant model.
    * Compares theoretical response vs. real-world data (CSV logs).
    * Used for tuning PID gains before deploying to hardware.

## 📊 Results

The controller achieves stable position tracking with minimal overshoot. The IIR filter effectively smooths the discrete encoder differentiation noise.

*(You can add a screenshot of the Python plot here)*

## 🔧 How to Run

1.  **Hardware Setup:** Connect the DC Motor driver and Encoders to the Nucleo pins as listed in the table.
2.  **Flash:** Open the project in **STM32CubeIDE** and flash the target.
3.  **Monitor:**
    * Open a Serial Terminal (Putty/RealTerm).
    * Baudrate: **115200**.
    * Format: `POS: 3.14 | REF: 3.14 | SPD: 0.05 | PWM: 1200`
4.  **Control:** Rotate the "Master" encoder; the motor should follow the position immediately.

## 📜 License
This project is open-source.
