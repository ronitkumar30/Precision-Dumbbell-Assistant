/* This file handles Bluetooth Low Energy (BLE) communication between the ESP32 and the web-based app. */

/* Initialize BLE Service */
void init_ble_service() {
    BLEDevice::init("PrecisionDumbbell");
    BLEServer* pServer = BLEDevice::createServer();
    BLEService* pService = pServer->createService(SERVICE_UUID);
    
    // Set up characteristic for feedback transmission
    BLECharacteristic* pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_NOTIFY
    );
    pService->start();
    BLEDevice::startAdvertising();
    Serial.println("Waiting for a client to connect...");
}
