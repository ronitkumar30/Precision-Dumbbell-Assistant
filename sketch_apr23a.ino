#include "SPI.h"
#include "lsm6dsm_reg.h"                   // Include library for standard BLE descriptor           
// #include <Arduino.h>                       // Include Arduino base library
#include "BluetoothSerial.h"               // Include library for Bluetooth serial communication (not used in this sketch)
#include <BLEDevice.h>                     // Include library to handle BLE devices
#include <BLEServer.h>                     // Include library to create BLE server
#include <BLEUtils.h>                      // Include utility library for BLE functionalities
#include <BLE2902.h>                       // Include library for standard BLE descriptor

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b" // Define the UUID for the BLE service
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8" // Define the UUID for the BLE characteristic

BLEServer* pServer = nullptr;             // Pointer to BLE server instance
BLECharacteristic* pCharacteristic = nullptr; // Pointer to BLE characteristic instance
bool deviceConnected = false;             // Flag for device connection status
bool oldDeviceConnected = false;          // Flag to track previous device connection status
int bicepCurls = 0;                       // Variable to store data to be sent over BLE
String curlFeedback = "";                 // Feedback on user form
int repetitions = -1;
float prevPitch = 0.0;
bool isCurlingUp = true;
bool isCurlingDown = false;
String feedback = "";

const int PWM_CHANNEL = 0;    // ESP32 has 16 channels which can generate 16 independent waveforms
const int PWM_FREQ = 200;     // Recall that Arduino Uno is ~490 Hz. Official ESP32 example uses 5,000Hz
const int PWM_RESOLUTION = 13;

const int MAX_DUTY_CYCLE = (int)(pow(2, PWM_RESOLUTION) - 1);
const int HALF_DUTY_CYCLE = MAX_DUTY_CYCLE / 2;

const int BUZZER_OUTPUT_PIN = 4; //4

const int SPI_CLK = 5*1000*1000; // 8Mhz
const int SPI_MOSI = 13; //13
const int SPI_MISO = 11; //11
const int SPI_SCK = 12; //12
const int SPI_CS_ONE = 10; //10
const int SENSOR_SEL = 45; //45

// const int SPI_CLK = 5*1000*1000; // 8Mhz
// const int SPI_MOSI = 4; //13
// const int SPI_MISO = 5; //11
// const int SPI_SCK = 2; //12
// const int SPI_CS_ONE = 15; //10
// const int SENSOR_SEL = 45; //45

uint8_t id;

static int16_t data_raw_acceleration[3];
static int16_t data_raw_angular_rate[3];
static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float orientation_deg[3];
static float accel_orientation[3];
static uint8_t whoamI, rst, initialized;

SPIClass dev_spi = SPIClass(HSPI);

stmdev_ctx_t dev_ctx;

class MyServerCallbacks: public BLEServerCallbacks { // Define custom server callback class inheriting from BLEServerCallbacks
    void onConnect(BLEServer* pServer) {  // Callback when device connects
      deviceConnected = true;             // Set the device connected flag
    };
    void onDisconnect(BLEServer* pServer) { // Callback when device disconnects
      deviceConnected = false;            // Reset the device connected flag
    }
};

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
    dev_spi.beginTransaction(SPISettings(SPI_CLK, MSBFIRST, SPI_MODE0));
    digitalWrite(SPI_CS_ONE, LOW);
    dev_spi.transfer(reg | 0x80);
    /* Read the data */
    for (uint16_t i=0; i<len; i++) {
      *(bufp+i) = dev_spi.transfer(0x00);
    }      
    digitalWrite(SPI_CS_ONE, HIGH);
    dev_spi.endTransaction();
    return 0;
}

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
    dev_spi.beginTransaction(SPISettings(SPI_CLK, MSBFIRST, SPI_MODE0));
    digitalWrite(SPI_CS_ONE, LOW);
    dev_spi.transfer(reg);
    /* Write the data */
    for (uint16_t i=0; i<len; i++) {
      dev_spi.transfer(bufp[i]);
    }
    digitalWrite(SPI_CS_ONE, HIGH);
    dev_spi.endTransaction();
    return 0;
}

void setup() {
  //printf("Waiting a client connection to notify...\r\n");
  Serial.begin(921600);                   // Start serial communication at 921600 baud rate
  BLEDevice::init("ESP32-Breadboard");               // Initialize the BLE device

  //printf("Waiting a client connection to notify...\r\n");
  pServer = BLEDevice::createServer();    // Create a BLE server
  pServer->setCallbacks(new MyServerCallbacks()); // Set custom callbacks for the server
  BLEService *pService = pServer->createService(SERVICE_UUID); // Create a BLE service
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ |
                      BLECharacteristic::PROPERTY_WRITE |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );                    // Create a BLE characteristic with multiple properties
  pCharacteristic->addDescriptor(new BLE2902()); // Add descriptor to characteristic
  pService->start();                      // Start the service
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising(); // Get the advertising object
  pAdvertising->addServiceUUID(SERVICE_UUID); // Add service UUID to the advertising data
  pAdvertising->setScanResponse(true);    // Set scan response
  pAdvertising->setMinPreferred(80);    // Set minimum preferred connection interval
  pAdvertising->setMinPreferred(160);    // Set maximum preferred connection interval
  pAdvertising->setMinPreferred(0x100);
  pAdvertising->setMinPreferred(0x2A0);
  BLEDevice::startAdvertising();          // Start advertising
  printf("Waiting a client connection to notify...\r\n"); // Print message to serial



  // // Set up pointers to read and write functions
  dev_ctx.write_reg = (stmdev_write_ptr)platform_write;
  dev_ctx.read_reg = (stmdev_read_ptr)platform_read;

  // Set up buzzer
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(BUZZER_OUTPUT_PIN, PWM_CHANNEL);

  //Set up SPI
  dev_spi.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SPI_CS_ONE);

  // Set up CS and Sensor select
  pinMode(SPI_CS_ONE, OUTPUT);
  pinMode(SENSOR_SEL, OUTPUT);
  digitalWrite(SENSOR_SEL, HIGH); 
  digitalWrite(SPI_CS_ONE, HIGH); 

  delay(20);

  // Handshake with sensor (Will need to be repeated for each sensor)
  lsm6dsm_device_id_get(&dev_ctx, &whoamI);
  printf("ID: %X\n", whoamI);
  if (whoamI != LSM6DSM_ID){
    // Turn on buzzer
    ledcWrite(PWM_CHANNEL, HALF_DUTY_CYCLE);
    printf("FAIL\n");
    while(1);
  }

  /* Restore default configuration */
  lsm6dsm_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
      lsm6dsm_reset_get(&dev_ctx, &rst);
  } while (rst);

    /*  Enable Block Data Update */
    lsm6dsm_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    /* Set Output Data Rate for Acc and Gyro */
    lsm6dsm_xl_data_rate_set(&dev_ctx, LSM6DSM_XL_ODR_52Hz);
    lsm6dsm_gy_data_rate_set(&dev_ctx, LSM6DSM_GY_ODR_52Hz);
    /* Set full scale */
    lsm6dsm_xl_full_scale_set(&dev_ctx, LSM6DSM_2g);
    lsm6dsm_gy_full_scale_set(&dev_ctx, LSM6DSM_500dps);
    /* Configure filtering chain(No aux interface)
    * Accelerometer - analog filter
    */
    lsm6dsm_xl_filter_analog_set(&dev_ctx, LSM6DSM_XL_ANA_BW_400Hz);
    /* Accelerometer - LPF1 path (LPF2 not used) */
    //lsm6dsm_xl_lp1_bandwidth_set(&dev_ctx, LSM6DSM_XL_LP1_ODR_DIV_4);
    /* Accelerometer - LPF1 + LPF2 path */
    lsm6dsm_xl_lp2_bandwidth_set(&dev_ctx,
                                LSM6DSM_XL_LOW_NOISE_LP_ODR_DIV_100);
    /* Accelerometer - High Pass / Slope path */
    //lsm6dsm_xl_reference_mode_set(&dev_ctx, PROPERTY_DISABLE);
    //lsm6dsm_xl_hp_bandwidth_set(&dev_ctx, LSM6DSM_XL_HP_ODR_DIV_100);
    /* Gyroscope - filtering chain */
    // lsm6dsm_gy_band_pass_set(&dev_ctx, LSM6DSM_HP_260mHz_LP1_STRONG);
    
    // Wait for IMU to stabilize
    delay(1000);
}

void loop() {
  if (deviceConnected) {
    //digitalWrite(LED_BUILTIN, HIGH);      // Turn on built-in LED if device is connected
    // while(deviceConnected) {              // Keep looping while device is connected
    float shoulderRoll = 0;
    float shoulderPitch = 0;
    float shoulderYaw = 0;
    float upperArmRoll = 0;
    float upperArmPitch = 0;
    float upperArmYaw = 0;
    float lowerArmRoll = orientation_deg[1];
    float lowerArmPitch = orientation_deg[0];
    float lowerArmYaw = orientation_deg[2];
    bicepCurls = repetitions;                       // Variable to store data to be sent over BLE
    curlFeedback = feedback; 

    String dataString = String(shoulderRoll) + "," + String(shoulderPitch) + "," + String(shoulderYaw) + "," + String(upperArmRoll) + "," + String(upperArmPitch) + "," + String(upperArmYaw) + "," + String(lowerArmRoll) + "," + String(lowerArmPitch) + "," + String(lowerArmYaw) + "," + String(bicepCurls) + "," + curlFeedback;
    Serial.println(dataString);
    pCharacteristic->setValue(dataString.c_str());  // Set the characteristic value
    pCharacteristic->notify();          // Notify connected device
      // delay(1000);
    // }
  }
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);                           // Delay for 500 milliseconds
    pServer->startAdvertising();          // Restart advertising
    // Serial.println("start advertising");  // Print message to serial
    oldDeviceConnected = deviceConnected; // Update old device connected status
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected; // Update old device connected status if newly connected
  }

  // put your main code here, to run repeatedly:
  lsm6dsm_reg_t reg;
  lsm6dsm_status_reg_get(&dev_ctx, &reg.status_reg);
  if (reg.status_reg.xlda) {
    /* Read acceleration field data */
    memset(data_raw_acceleration, 0x00, 3 * sizeof(int16_t));
    lsm6dsm_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
    acceleration_mg[0] = lsm6dsm_from_fs2g_to_mg(data_raw_acceleration[0]);
    acceleration_mg[1] = lsm6dsm_from_fs2g_to_mg(data_raw_acceleration[1]);
    acceleration_mg[2] = lsm6dsm_from_fs2g_to_mg(data_raw_acceleration[2]);
    accel_orientation[0] = atan2f(acceleration_mg[1], acceleration_mg[2]) * 180.0/M_PI;
    if(accel_orientation[0] < 0.0){
      accel_orientation[0] += 360.0;
    }
    if(accel_orientation[0] > 360.0){
      accel_orientation[0] -= 360.0;
    }
    accel_orientation[1] = atan2f(acceleration_mg[0], acceleration_mg[2]) * 180.0/M_PI;
    accel_orientation[2] = atan2f(acceleration_mg[0], acceleration_mg[1]) * 180.0/M_PI;
    // printf("Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
    //         acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
    // printf("Accel Pitch [deg]: %4.2f\t Accel Roll [deg]: %4.2f\r\n", accel_orientation[0], accel_orientation[1]);
  }else{
    // printf("FAIL................\r\n");
  }

  // Set initial orientation state
  if(!initialized){
    orientation_deg[0] = accel_orientation[0];
    orientation_deg[1] = accel_orientation[1];
    initialized = 1;
  }

  if (reg.status_reg.gda) {
    /* Read angular rate field data */
    memset(data_raw_angular_rate, 0x00, 3 * sizeof(int16_t));
    lsm6dsm_angular_rate_raw_get(&dev_ctx, data_raw_angular_rate);
    angular_rate_mdps[0] =
        lsm6dsm_from_fs500dps_to_mdps(data_raw_angular_rate[0]);
    angular_rate_mdps[1] =
        lsm6dsm_from_fs500dps_to_mdps(data_raw_angular_rate[1]);
    angular_rate_mdps[2] =
        lsm6dsm_from_fs500dps_to_mdps(data_raw_angular_rate[2]);
    // printf("Gyro Pitch [deg]: %4.2f\t Gyro Roll [deg]: %4.2f\t Gyro Yaw [deg]: %4.2f\r\n", angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
    // Complementary filter to avoid drift (98% gyro data, 2% accel data), 0.02 sec timestep, this value can be tuned once we know how long it takes the loop to run
    orientation_deg[0] = 0.98*(orientation_deg[0] + ((angular_rate_mdps[0] / 1000) * 0.02) + 0.02*accel_orientation[0]);
    // Set range to 0 to 360 instead of -180 to 180
    if(orientation_deg[0] < 0.0){
      orientation_deg[0] += 360.0;
    }
    if(orientation_deg[0] > 360.0){
      orientation_deg[0] -= 360.0;
    }
    orientation_deg[1] = 0.98*(orientation_deg[1] + ((angular_rate_mdps[1] / 1000) * 0.02) - 0.02*accel_orientation[1]);
    orientation_deg[2] = 0.98*(orientation_deg[2] + ((angular_rate_mdps[2] / 1000) * 0.02) + 0.02*accel_orientation[2]);
    printf("Gyro Pitch [deg]: %4.2f\t Gyro Roll [deg]: %4.2f\t Gyro Yaw [deg]: %4.2f\r\n", orientation_deg[0], orientation_deg[1], orientation_deg[2]);
    // printf("Angular rate [mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
    //         angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
    // tx_com(tx_buffer, strlen((char const *)tx_buffer));
  }


  // Repetitions
  const float lowerrollThreshold = -175.0;
  const float upperrollThreshold = 175.0;
  const float loweryawThreshold = -175.0;
  const float upperyawThreshold = 10.0;
  const float lowerPitchThreshold = 125.0;
  const float upperPitchThreshold = 175.0;

  if (orientation_deg[0] > upperPitchThreshold && isCurlingUp){
    repetitions++;
    isCurlingUp = false;
  }
  if (orientation_deg[0] < lowerPitchThreshold){
    isCurlingUp = true;
  }

  //DO FORM ANALYSIS HERE (LOWER ARM):
  // (-76, 86, 7) - hand at bottom
  // (-165, 166, 46) - hand in middle
  // (-169, 224, 167) - hand at top
  
  //check for pushing arm back at bottom
  if (orientation_deg[0] < 75){
    feedback = "Avoid pushing your arm too far back at bottom. Don't swing weights. Use only biceps.";
  }
  //check for pulling arm at top
  else if (orientation_deg[0] > 250){
    feedback = "Keep elbow aligned with body. Don't move it forward.";
    // Buzzer on
    // ledcWrite(PWM_CHANNEL, HALF_DUTY_CYCLE);
  }
  else{
    feedback = "Good form!";
  }

  // const float biceprollThreshold = 100.0; // Similar to lower arm since the upper arm shouldn't rotate much either
  // const float bicepyawThreshold = 50.0;   // Similar to lower arm to avoid lateral movements
  // const float biceplowerPitchThreshold = -50.0;  // Smaller range, expecting less dramatic pitch changes
  // const float bicepupperPitchThreshold = 50.0;   // Smaller range, expecting less dramatic pitch changes

  // if ((orientation_deg[2] < loweryawThreshold) || (orientation_deg[2] > upperyawThreshold)){ 
  //   feedback = "Align your elbow with your body. Avoid swinging the arm sideways.";
  //   // Buzzer on
  //   // ledcWrite(PWM_CHANNEL, HALF_DUTY_CYCLE);
  // }
  // else if ((orientation_deg[1] < lowerrollThreshold) || (orientation_deg[1] > upperrollThreshold)){
  //   feedback = "Keep your wrist straight. Avoid rotating your forearm.";
  //   // Buzzer on
  //   // ledcWrite(PWM_CHANNEL, HALF_DUTY_CYCLE);
  // }
  // else if (orientation_deg[0] < 75){
  //   feedback = "Avoid overextending downwards.";
  //   // Buzzer on
  //   // ledcWrite(PWM_CHANNEL, HALF_DUTY_CYCLE);
  // }
  // else if (orientation_deg[0] > 250){
  //   feedback = "Avoid overextending upwards.";
  //   // Buzzer on
  //   // ledcWrite(PWM_CHANNEL, HALF_DUTY_CYCLE);
  // }
  // else if (abs(biceporientation_deg[1]) > rollThreshold) {
  //   feedback = "Keep your elbow steady. Avoid rotating your upper arm.";
  //   // Buzzer on
  //   // ledcWrite(PWM_CHANNEL, HALF_DUTY_CYCLE);
  // }
  // else if (abs(biceporientation_deg[2]) > yawThreshold) {
  //   feedback = "Align your elbow with your body. Avoid swinging the arm sideways.";
  //   // Buzzer on
  //   // ledcWrite(PWM_CHANNEL, HALF_DUTY_CYCLE);
  // }
  // else if (biceporientation_deg[0] < lowerPitchThreshold) {
  //   feedback = "Keep upper arm steady; avoid dropping it too low.";
  //   // Buzzer on
  //   // ledcWrite(PWM_CHANNEL, HALF_DUTY_CYCLE);
  // }
  // else if (biceporientation_deg[0] > upperPitchThreshold) {
  //   feedback = "Keep upper arm steady; avoid raising it too high.";
  //   // Buzzer on
  //   // ledcWrite(PWM_CHANNEL, HALF_DUTY_CYCLE);
  // }
  // else{
  //   feedback = "Good form!";
  //   // Buzzer off
  //   // ledcWrite(PWM_CHANNEL, 0);
  // }

  //Check pitch
  if(orientation_deg[0] < 90.0 || orientation_deg[0] > 270.0){
    // Buzzer on
    ledcWrite(PWM_CHANNEL, HALF_DUTY_CYCLE);
  }else{
    // Buzzer off
    ledcWrite(PWM_CHANNEL, 0);
  }
}
