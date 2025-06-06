# Seesaw Test Bench: Modeling and Control

## 📝 Overview

This project details the development, modeling, and control of a "Seesaw" laboratory test bench. The system is a Two-Input, Single-Output (TISO) platform inspired by bicopter drone dynamics, where two motors are used to manipulate the tilt angle of a central beam.

The primary objective is to present the physical prototype, derive its mathematical model (both non-linear and linearized), and implement and compare four different control strategies to manage the seesaw's angular position. The performance of these controllers is evaluated in both simulation and through experimental tests on the physical hardware.

---

## 🛠️ Hardware and Implementation

### Physical Setup
* **Actuators:** Two drone motors with propellers provide the input torques.
* **Sensor:** A potentiometer is connected to the central rotation axis to measure the tilt angle.
* **Interface:** An **Arduino UNO** board is used to interface with the motors and sensor.
* **Driver:** A dedicated motor driver board controls the speed of the motors.

### Implementation Method
The control algorithms were developed in **Simulink** and deployed using the **Connected I/O** mode. In this configuration, the Arduino UNO functions solely as a hardware interface for inputs and outputs. All signal processing and the execution of the complex control laws occur in real-time within the Simulink environment on the host computer. This approach effectively bypasses the limited processing power of the Arduino board, allowing for the testing of advanced control strategies.

### State Estimation
Since only the angular position is measured directly, the angular velocity is estimated. To overcome the high noise level from a simple derivative calculation, a **Luenberger Observer** was designed and implemented to provide a smoother and more reliable velocity estimation for the state-feedback controllers.

---

## 🚀 Control Strategies Implemented

Four distinct control strategies were designed and compared:

1.  **Discrete PID Controller:** A classical filtered Proportional-Integral-Derivative controller designed based on a simplified linear model of the system.
2.  **RST Controller (2-DOF):** A polynomial controller designed for discrete-time systems. It offers the flexibility to separately define tracking and regulation dynamics and was configured with an integrator to ensure zero steady-state error.
3.  **State Feedback (SF):** This approach uses an extended state-space model (including an integrator state) to provide robust disturbance rejection. The controller gains were calculated using the pole placement technique to achieve the desired closed-loop dynamics.
4.  **Feedback Linearization + State Feedback (FL+SF):** A non-linear control technique that uses the inverse of the system's non-linear model to cancel out its dynamics, resulting in a simple linear system (a chain of integrators). This linearized system is then controlled using an extended state-feedback controller.

---

## 📊 Performance Analysis & Key Findings

The controllers were evaluated in both simulation and experimental tests, facing reference tracking tasks and disturbances. Performance was quantitatively measured using the IAE, ISE, ITAE, and ITSE error metrics.

### General Performance
* The state-based controllers, **State Feedback (SF)** and **Feedback Linearization + State Feedback (FL+SF)**, consistently demonstrated the best performance in both simulation and experimental tests. They resulted in the lowest accumulated errors, indicating fast, precise control with minimal overshoot.
* The **RST** controller provided solid, intermediate performance, consistently outperforming the PID.
* The **PID** controller showed the worst performance across all scenarios and metrics, exhibiting significantly higher errors.

### Robustness to Dead-Zone and Delay
The controllers were also tested against actuator dead-zone and signal delay.
* **Dead-Zone:** The SF and FL+SF controllers proved to be highly robust to actuator dead-zones, maintaining good performance even with large non-linearities. The PID and RST controllers, in contrast, showed significant performance degradation.
* **Delay:** In the presence of signal delay, the **RST controller** was surprisingly the most robust. The state-based controllers (SF and FL+SF) were more sensitive to delay and showed the potential for instability with larger delays.
