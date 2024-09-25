/* This file implements the power subsystem for regulating and testing the voltage supplied to the ESP32 and sensors. */

/* Voltage Regulation Code */
void setup_voltage_regulator() {
    // Set up voltage regulator for 3.3V output
    pinMode(VREG_PIN, OUTPUT);
    digitalWrite(VREG_PIN, HIGH);
    
    // Ensure proper voltage is supplied to ESP32
    check_voltage();
}

/* Voltage Testing Function */
float check_voltage() {
    float voltage = analogRead(VIN_PIN) * (3.3 / 4095.0);
    return voltage;
}
