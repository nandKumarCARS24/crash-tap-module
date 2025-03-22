#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertising.h>
#include <BLEAdvertisedDevice.h>
#include <MFRC522.h>

#define PIEZO_PIN 34                 // GPIO pin for piezo sensor
#define VEHICLE_ID "HR26DA1234"      // Unique vehicle ID
#define MANUFACTURER_ID 0x1234       // Custom BLE manufacturer ID
#define IMPACT_THRESHOLD 1000        // Collision threshold
#define BROADCAST_DURATION 50000      // BLE TX/RX duration in ms

#define SS_PIN  5     // SDA pin of RC522 to GPIO 5
#define RST_PIN 22 

BLEAdvertising* pAdvertising;
BLEScan* pScanner;
bool collisionActive = false;

MFRC522 mfrc522(SS_PIN, RST_PIN);  // Declare once, top of file

void writeToRFID(String data) {
  if (!mfrc522.PICC_IsNewCardPresent() || !mfrc522.PICC_ReadCardSerial()) {
    Serial.println("⚠️ No RFID tag present");
    return;
  }

  byte block = 4;  // Block 4 is safe to write
  byte buffer[16];

  // Fill buffer with vehicle ID or pad with spaces
  for (int i = 0; i < 16; i++) {
    if (i < data.length()) buffer[i] = data[i];
    else buffer[i] = ' ';
  }

  // Default key for MIFARE Classic
  MFRC522::MIFARE_Key key;
  for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;

  if (mfrc522.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, block, &key, &(mfrc522.uid)) != MFRC522::STATUS_OK) {
    Serial.println("❌ Authentication failed");
    return;
  }

  MFRC522::StatusCode status = mfrc522.MIFARE_Write(block, buffer, 16);
  if (status == MFRC522::STATUS_OK) {
    Serial.println("✅ Vehicle ID written to RFID tag");
  } else {
    Serial.println("❌ Failed to write to RFID");
  }

  mfrc522.PICC_HaltA();
  mfrc522.PCD_StopCrypto1();
}


// BLE RX callback
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    String raw = advertisedDevice.getManufacturerData();
    if (raw.length() > 2) {
      uint16_t id = (uint8_t)raw[1] << 8 | (uint8_t)raw[0];
      if (id == MANUFACTURER_ID) {
        String vehicleID = raw.substring(2);
        writeToRFID(vehicleID);
        // Serial.print("📡 Received vehicle ID: ");
        Serial.println(vehicleID);
      }
    }
  }
};


void startBLEBroadcastAndScan() {
  Serial.println("📢 Broadcasting vehicle ID and scanning nearby devices...");

  String payload;
  payload += (char)(MANUFACTURER_ID & 0xFF);         // LSB
  payload += (char)((MANUFACTURER_ID >> 8) & 0xFF);  // MSB
  payload += VEHICLE_ID;

  BLEAdvertisementData advData;
  advData.setManufacturerData(payload);
  pAdvertising->setAdvertisementData(advData);
  pAdvertising->start();

  pScanner->clearResults();
  pScanner->start(BROADCAST_DURATION / 1000, false);  // in seconds

  delay(BROADCAST_DURATION);

  pAdvertising->stop();
  pScanner->stop();

  Serial.println("✅ BLE TX/RX cycle complete.\n");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIEZO_PIN, INPUT);

  BLEDevice::init("VehicleCollisionNode");

  // BLE Advertiser
  pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);

  // BLE Scanner
  pScanner = BLEDevice::getScan();
  pScanner->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pScanner->setInterval(100);
  pScanner->setWindow(90);
  pScanner->setActiveScan(true);

  Serial.println("🚗 System ready. Waiting for collision...");
}

void loop() {
  int sensorValue = analogRead(PIEZO_PIN);
  if (sensorValue > IMPACT_THRESHOLD && !collisionActive) {
    Serial.println("🚨 Collision Detected! Value: " + String(sensorValue));
    collisionActive = true;

    startBLEBroadcastAndScan();

    collisionActive = false;
  }

  delay(200);  // Debounce and reduce CPU load
}