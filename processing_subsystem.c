/* This file handles sensor data processing and form analysis. */

/* Form Analysis Code */
void analyze_form(float pitch, float roll, float yaw) {
    if (pitch > MAX_PITCH || pitch < MIN_PITCH) {
        give_feedback("Incorrect form! Adjust your arm position.");
    } else {
        give_feedback("Good form, keep going!");
    }
}

/* Feedback Function */
void give_feedback(const char* message) {
    // Send feedback to user via buzzer or web app
    Serial.println(message);
    activate_buzzer_if_needed(message);
}
