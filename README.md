# EDF-TVC-Test-Bench

This repository contains the software and testing environments for an Electric Ducted Fan (EDF) Thrust Vector Control (TVC) Test Bench. The system utilizes an Arduino Uno, FreeRTOS for task management, and implements a Tustin PD controller to drive gimbal servos based on encoder feedback.

## Hardware Components
- **Microcontroller**: Arduino Uno (Atmel AVR)
- **Actuation**: Gimbal Servos (e.g., MG90S)
- **Sensing**: Quadrature Encoders

## Software Dependencies
This project is built using [PlatformIO](https://platformio.org/). The following libraries are used:
- `arduino-libraries/Servo`
- `feilipu/FreeRTOS`

## Repository Structure

This project uses a multi-environment PlatformIO configuration to separate the main control code from individual component tests. You can build and upload specific environments using the PlatformIO sidebar or the CLI.

### Environments
- **`env:main`**: The core TVC controller utilizing FreeRTOS tasks to orchestrate sensing and actuation. 
  - Source: `src/main_code/`
- **`env:test_encoder`**: Isolated testing for the custom quadrature encoder ISR.
  - Source: `src/test_encoder/`
- **`env:test_servo`**: Isolated testing for servo movement and pulse width calibration.
  - Source: `src/test_servo/`
- **`env:test_step`**: High-frequency step response testing for characterizing gimbal servo dynamics.
  - Source: `src/test_step/`


## Getting Started

1. **Install PlatformIO**: Install the PlatformIO extension for VSCode.
2. **Open the Project**: Open the `EDF_TVC` folder in VSCode.
3. **Select an Environment**: Click the PlatformIO target selector in the bottom taskbar (or use `pio run -e <env_name>`) to build/upload specific tests or the main controller.
4. **Upload**: Connect your Arduino Uno and click the Upload button (right arrow).

## Data Logging
The project includes a Python logger (`src/logger/logger.py`) designed to read serial output (like step response data) and save it to `logs/tvc_flight_data.csv` for analysis.
