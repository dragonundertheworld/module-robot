# Self-Balancing Seesaw Robot
- [Self-Balancing Seesaw Robot](#self-balancing-seesaw-robot)
  - [1. Project Overview](#1-project-overview)
  - [2. System Design](#2-system-design)
    - [2.1 Control System Architecture](#21-control-system-architecture)
    - [2.2 Mechanical Structure](#22-mechanical-structure)
    - [2.3 Sensing System](#23-sensing-system)
    - [2.4 Actuation System](#24-actuation-system)
  - [3. Control Strategy](#3-control-strategy)
    - [3.1 PID Control Algorithm](#31-pid-control-algorithm)
    - [3.2 Sensor Data Filtering](#32-sensor-data-filtering)
    - [3.3 Control Flow Logic](#33-control-flow-logic)
  - [4. Software Implementation](#4-software-implementation)
  - [5. Hardware Setup \& Wiring](#5-hardware-setup--wiring)
  - [6. Testing \& Results](#6-testing--results)
  - [7. Future Improvements](#7-future-improvements)

## 1. Project Overview

This project aims to design and build a robot capable of autonomously navigating onto a seesaw, balancing on it, and returning to its starting position. The core challenge lies in maintaining stability on an unstable platform using sensor feedback and control algorithms.

**Core Objectives:**

1.  **Initial Alignment:** The robot must automatically correct its orientation (Yaw angle) to align with the seesaw from its starting position.
2.  **Seesaw Navigation:** Autonomously drive onto the seesaw.
3.  **Balancing:** Find and maintain a stable equilibrium point on the seesaw for approximately 10 seconds, primarily by controlling the Pitch angle.
4.  **Return:** Safely drive back off the seesaw to the starting area.
5.  **(Optional)** Maintain balance even when additional weight (10-20% of the robot's weight) is added to one end of the seesaw.
6.  **(Optional)** Maintain balance with a rolling ball placed on the seesaw.

## 2. System Design

### 2.1 Control System Architecture

The robot employs a closed-loop control system. A 9-axis sensor provides real-time orientation data (Pitch and Yaw angles). This data is processed by an STM32 microcontroller running a PID control algorithm. The controller outputs signals to the motor drivers to adjust the robot's speed and direction, thereby correcting its orientation and maintaining balance.

![Control System Diagram](https://github.com/dragonundertheworld/module-robot/blob/main/img/control_system.jpg)
*Figure 1: Robot Control System Block Diagram*

### 2.2 Mechanical Structure

*   **Chassis:** A rectangular frame (~20cm x 13cm) built using aluminum profiles.
*   **Wheel Configuration:** A 3-wheel design was chosen over a 4-wheel setup to reduce turning friction and simplify construction. It features two rear drive wheels and a single front omni-directional wheel for support.
*   **Component Layout:**
    *   **Drive Motors (Servos):** Mounted at the rear, connected to the wheels via couplings.
    *   **Battery:** Placed at the front to help counterbalance the rear motors and improve stability.
    *   **Controller (STM32) & Breadboard:** Located towards the rear.
    *   **Sensor (GY-953):** Mounted at the front, above the omni-wheel, positioned away from potential electromagnetic interference from motors and power lines.

### 2.3 Sensing System

*   **Sensor:** GY-953 9-axis Inertial Measurement Unit (IMU), containing a 3-axis gyroscope, 3-axis accelerometer, and 3-axis magnetometer.
*   **Data Acquisition:** Provides Euler angles (Roll, Pitch, Yaw) via serial (UART) communication at a frequency of 50Hz.
    *   **Yaw Angle:** Used for heading correction and alignment.
    *   **Pitch Angle:** Crucial for balancing on the seesaw.
*   **Communication:** The sensor communicates with the STM32 microcontroller via UART.
*   **Calibration:** The sensor is sensitive to magnetic interference and requires regular magnetic calibration for accurate Yaw readings.

### 2.4 Actuation System

*   **Actuators:** Two servo motors drive the rear wheels independently.
*   **Motion Control:**
    *   **Forward/Backward:** Both servos rotate in the same direction at the same speed.
    *   **Turning:** Servos rotate at different speeds (differential drive).
*   **Dead Zone Compensation:** A calibration function (`MotorCalib`) is implemented to compensate for the motor dead zones, ensuring more precise low-speed control.

## 3. Control Strategy

### 3.1 PID Control Algorithm

The core of the balancing and steering control is the Proportional-Integral-Derivative (PID) algorithm. A discrete form of the PID algorithm is used due to the digital nature of the sensor readings and controller.

![PID Formula](https://github.com/dragonundertheworld/module-robot/blob/main/img/PID.jpg)
*Figure 2: Discrete PID Control Formula*

*   **Error Calculation:** `error = current_angle - target_angle`
*   **PID Terms:**
    *   **Proportional (Kp):** Reacts to the current error. Increasing Kp generally leads to faster response but can cause overshoot and oscillation.
    *   **Integral (Ki):** Accumulates past errors to eliminate steady-state error. Helps the robot settle precisely at the target angle but can decrease stability if too large.
    *   **Derivative (Kd):** Reacts to the rate of change of the error. Dampens oscillations and improves stability by predicting future errors. Acts like a braking force.
*   **Tuning:**
    *   **Pitch Control:** A two-stage PID tuning approach was adopted. Different Kp, Ki, and Kd values are used depending on whether the Pitch error is large (>= 2 degrees) or small (< 2 degrees). This allows for aggressive correction when far from balance and finer adjustments when close to the target angle.
    *   **Yaw Control:** Primarily uses Kp and Kd terms for direction alignment. The Ki term was found less critical for this application.

### 3.2 Sensor Data Filtering

Raw sensor data from the GY-953 can be noisy due to vibrations and electromagnetic interference. To improve control stability, a simple **moving average filter** is applied to both Pitch and Yaw readings.

*   **Implementation:** The `CalculateAverage` function reads sensor data multiple times (e.g., 5 times), sums the readings, and divides by the number of samples to get an average value. A delay (`osDelay(20)`) corresponding to the sensor's update rate (50Hz -> 20ms) is inserted between readings.

### 3.3 Control Flow Logic

The main control task (`StartTaskYawCorrect`) orchestrates the robot's behavior:

1.  **Initialization:** Read sensor data and calculate initial average angles.
2.  **Initial Yaw Correction:** If the initial Yaw angle error is outside a threshold (e.g., > 2 degrees), rotate the robot using Yaw PID control until aligned.
3.  **Approach Seesaw:** Drive forward at a constant speed for a set duration (e.g., 6 seconds) or until a significant Pitch angle change indicates the robot is on the seesaw.
4.  **Balancing Phase:**
    *   Continuously read sensor data and apply filtering.
    *   Use Pitch PID control (`PitchPIDControl`) to adjust motor speeds (`SPEED`) to maintain balance (target Pitch angle).
    *   Periodically (e.g., every 4 seconds via `StartTaskMotorCtrl`), check and correct the Yaw angle using Yaw PID control (`YawPIDControl`) to prevent drifting off the sides.
5.  **Hold Balance:** Once the Pitch error is within a small tolerance (e.g., +/- 1.6 degrees) for a sustained period (counted by `array`), hold the position for the required time (e.g., 15 seconds).
6.  **Return:** Drive backward off the seesaw. (Note: The README mentions this part needs further refinement for better Yaw correction and stopping).

## 4. Software Implementation

*   **Microcontroller:** STM32F401CCU6 (based on `module-robot/first_try.ioc`)
*   **Development Environment:** Keil MDK-ARM (based on `.uvprojx` files)
*   **Real-Time Operating System (RTOS):** FreeRTOS is used for managing concurrent tasks.
    *   **Tasks:** Separate tasks handle sensor data acquisition (UART), motor control (PWM), PID calculations, and the main control logic. ([`StartTaskUART1GY953`](module-robot/Core/Src/main.c), [`StartTaskMotorCtrl`](module-robot/Core/Src/main.c), [`StartTaskPID`](module-robot/Core/Src/main.c), [`StartTaskYawCorrect`](module-robot/Core/Src/main.c)).
    *   **Synchronization:** Semaphores ([`myBinarySem_UART1GY953RecvHandle`](module-robot/Core/Src/main.c), [`myBinarySem_MotorSpdChangeHandle`](module-robot/Core/Src/main.c), etc.) are used to coordinate tasks, e.g., signaling when new sensor data is available or when PID calculations are ready.
*   **Libraries:** STM32 HAL (Hardware Abstraction Layer), CMSIS-RTOS API for FreeRTOS.
*   **Key Functions:**
    *   `MotorCalib(chn, spd)`: Applies dead zone correction and sends speed command to a motor channel.
    *   `CalculateAverage()`: Reads sensor data multiple times, calculates the average Pitch and Yaw, and releases a semaphore.
    *   `YawPIDInit()` / `YawPIDControl(YawAverage)`: Initializes and computes the Yaw PID control output (`YawCorrectionSpeed`).
    *   `PitchPIDControl(PitchAverage)`: Computes the Pitch PID control output (`SPEED`) and updates the global `PitchError`.

## 5. Hardware Setup & Wiring

*   The GY-953 sensor is connected to the STM32 via UART (USART1).
*   Servos are connected to STM32 PWM output pins (TIM2, CH1 & CH2).
*   Power is supplied by two 18650 batteries, regulated as needed for the microcontroller and servos.
*   A breadboard and/or breakout boards are used for connections.

![Circuit Diagram Sketch](https://github.com/dragonundertheworld/module-robot/blob/main/img/circuit_design.jpg)
*Figure 3: Simplified Circuit Connection Diagram*

## 6. Testing & Results

*   **Methodology:** Testing was performed in two stages: ground tests (checking rotation and straight-line movement) and seesaw tests (evaluating balancing performance).
*   **PID Tuning:** Parameters were iteratively tuned by observing the robot's behavior on the seesaw. Kp was adjusted first to achieve oscillation around the balance point, followed by Kd for damping, and finally Ki to minimize steady-state error. Specific tuned values are documented in the original README's tables.
*   **Challenges & Solutions:**
    *   **Noisy Sensor Data:** Addressed by implementing the moving average filter.
    *   **UART Communication Errors:** Resolved by adding appropriate delays (`osDelay`) in the communication tasks.
    *   **Inaccurate Motor Control:** Corrected by implementing dead zone compensation (`MotorCalib`) and ensuring it was used consistently.
    *   **Drifting Off Seesaw:** Mitigated by adding a separate task (`StartTaskMotorCtrl`) for periodic Yaw correction during balancing.
    *   **Slow Approach:** Improved by driving forward for a fixed duration initially.
    *   **Debugging:** Using onboard LED blinking proved helpful for verifying code execution flow.

## 7. Future Improvements

*   Implement a more sophisticated filter (e.g., Kalman filter or FIR filter) for potentially better noise rejection.
*   Improve Yaw correction during balancing, possibly by integrating magnetometer data more effectively or using sensor fusion.
*   Refine the backward movement logic for better directional control and accurate stopping.
*   Enhance robustness against external disturbances.
