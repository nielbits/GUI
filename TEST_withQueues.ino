#include <Arduino.h>
#include <NimBLEDevice.h>
#include <string>
#include "queues.h"

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

NimBLECharacteristic *pTxCharacteristic = nullptr;
bool deviceConnected = false;

#define VESC_RX_PIN 4
#define VESC_TX_PIN 5
#define VESC_BAUD   115200

queue_uint8 q_ser;
queue_uint8 q_ble;
packet p_ser;
packet p_ble;

class MyServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
    deviceConnected = true;
    Serial.println("[BLE STATUS] Client Connected!");
  }

  void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
    deviceConnected = false;
    Serial.printf("[BLE STATUS] Client Disconnected, reason=%d\n", reason);
    delay(300);
    NimBLEDevice::startAdvertising();
    Serial.println("[BLE STATUS] Advertising restarted");
  }
};

class TxCallbacks : public NimBLECharacteristicCallbacks {
  void onSubscribe(NimBLECharacteristic* pCharacteristic,
                   NimBLEConnInfo& connInfo,
                   uint16_t subValue) override {
    Serial.printf("[BLE TX] subscribe state=%u\n", subValue);
  }
};

class MyCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo& connInfo) override {
    std::string rxValue = pCharacteristic->getValue();

    if (!rxValue.empty()) {
      for (size_t i = 0; i < rxValue.length(); i++) {
        packet_handler(&q_ble, &p_ble, (uint8_t)rxValue[i], BLE, pTxCharacteristic);
      }
    }
  }
};

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println();
  Serial.println("[BOOT] Starting queue-based NimBLE VESC bridge");

  Serial1.begin(VESC_BAUD, SERIAL_8N1, VESC_RX_PIN, VESC_TX_PIN);

  queue_init(&q_ser);
  packet_init(&p_ser);
  queue_init(&q_ble);
  packet_init(&p_ble);

  while (Serial1.available() > 0) {
    Serial1.read();
  }

  NimBLEDevice::init("VESC_BLE_BRIDGE");
  NimBLEDevice::setMTU(185);

  NimBLEServer *pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  NimBLEService *pService = pServer->createService(SERVICE_UUID);

  pTxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_TX,
    NIMBLE_PROPERTY::NOTIFY
  );
  pTxCharacteristic->setCallbacks(new TxCallbacks());

  NimBLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID_RX,
    NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR
  );
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();

  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->enableScanResponse(true);
  pAdvertising->setName("VESC_BLE_BRIDGE");

  NimBLEDevice::startAdvertising();

  Serial.println("[BOOT] BLE ready");
  Serial.println("[BOOT] Waiting for BLE peer...");
}

void loop() {
  while (Serial1.available() > 0) {
    uint8_t b = (uint8_t)Serial1.read();
    packet_handler(&q_ser, &p_ser, b, SER, pTxCharacteristic);
  }

  delay(1);
}