This project demonstrates the implementation of a basic STM32-based code for a high-powered rocketry flight computer, utilizing the STM32F429 microcontroller. The system is designed to test the functionality of an RTOS alongside hardware components such as a barometric pressure sensor and a Bluetooth module.

Key Features:
RTOS-Based Multi-threading:

Thread 1: Acquires data from the barometric pressure sensor.
Thread 2: Transmits the acquired data to a connected device via Bluetooth for communication purposes.
Interrupt-Driven Telecommand System:

Supports the retrieval of telecommands for:
Essential health checks.
Sensor calibration.
Actuator functionality verification.
This setup is a prototype aimed at validating the integration of hardware and software for rocketry applications. It ensures real-time data acquisition, communication, and remote system management capabilities essential for high-powered rocketry missions.