# Precision-Dumbbell-Assistant

## Project Overview

The precision Dumbbell Assistant is a wearable device designed to assist users in maintaining proper form while performing bicep curls. It tracks the orientation of the user's arm in real-time using two 6-axis Inertial Measurement Unites (IMUs), providing feedback through a buzzer and web-based application.

### Main Features
- **Real-time form analysis:** Tracks arm orientation and alerts user with feedback
- **Repetition counting:** Accurately counts repetitions of bicep curls
- **Wireless communication:** Uses Bluetooth Low Energy (BLE) to send form data and feedback to web application
- **Compact and wearable design:** Includes 3D-printed housing for electronics, powered by rechargeable battery

### Project Files
- **power_subsystem.c:** Power supply design for system
- **processing_subsystem.c:** Handles sensor data processing and form analysis
- **ble_communication.c:** BLE communication code for transmitting data to app
- **sensing_subsystem.c:** Manages sensor data collection from IMUs

### Technologies Used
- ESP32 Microcontroller
- BLE (Bluetooth Low Energy)
- IMU (Inertial Measurement Units)
- Complimentary Filter for data fusion
- Web Application (HTML, CSS, JavaScript)
