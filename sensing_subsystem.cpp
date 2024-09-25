/* This file manages data collection from the IMUs and processes the accelerometer and gyroscope data. */

/* Process Sensor Data */
void process_sensor_data() {
    lsm6dsm_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
    acceleration_mg[0] = lsm6dsm_from_fs2g_to_mg(data_raw_acceleration[0]);
    acceleration_mg[1] = lsm6dsm_from_fs2g_to_mg(data_raw_acceleration[1]);
    
    // Use complementary filter to calculate orientation
    orientation_deg[0] = 0.98 * (orientation_deg[0] + angular_rate[0] * 0.01) + 0.02 * acceleration_mg[0];
}
