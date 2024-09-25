### Abstract

The Precision Dumbbell Assistant is a wearable fitness device designed to help users perform bicep curls with proper form while tracking repetitions. This device uses two 6-axis Inertial Measurement Units (IMUs) to track the orientation of the user's arm in real time. An ESP32 microcontroller processes this data and provides feedback through a buzzer and web-based app. The system is designed to improve home workouts by ensuring users maintain proper form without needing extensive gym equipment.

---

### Problem and Solution

#### Problem

Many home gym users lack guidance on maintaining proper form, which can lead to inefficient workouts and injuries. Traditional exercise machines in gyms provide support for correct form, but home workout setups often do not.

#### Solution

The Precision Dumbbell Assistant addresses this problem by using two IMUs to track arm movement and provide real-time feedback. When incorrect form is detected, the user is alerted via a buzzer and additional feedback is provided through a web-based app connected via BLE.

---

### Design

#### Power Subsystem

The system is powered by a rechargeable 3.7 V battery regulated down to 3.3 V for use by the ESP32 and sensors.

#### Processing Subsystem

The ESP32 microcontroller processes data from the IMUs using a complementary filter to calculate the orientation of the user's arms. Based on these calculations, the system detects if the user is performing the exercise correctly.

#### Sensing Subsystem

Two LSM6DSM IMU sensors are used to track both the upper and lower arm orientations in 3D space (pitch, roll, yaw).

#### Wireless Communication Subsystem

Bluetooth Low Energy (BLE) is used to transmit form data to a web-based app for detailed feedback. The app also tracks the number of repetitions.

---

### Testing and Results

**Orientation Accuracy:** Sensor orientation was tested with an accuracy of ±5%. Each axis of the IMUs was verified against manual measurements using an oscilloscope.

**Feedback Latency:** Feedback was generated within 50 ms of incorrect form detection. This met the requirement for providing real-time feedback at a minimum of 20 Hz.

**Wireless Range:** BLE communication was tested up to 5 meters without any packet loss, ensuring reliable data transmission.

**Buzzer Feedback:** The buzzer was set to provide audible feedback at 200 Hz, which was sufficiently loud at 65 bdB for a user to hear.

---

### Conclusion

The Precision Dumbbell Assistant successfully tracks bicep curl form and counts repetitions. The use of two IMUs enables accurate orientation tracking, and feedback is provided both audibly and via a web app. The project can be extended to other dumbbell exercises and potentially include more sensors for full-body tracking. 
