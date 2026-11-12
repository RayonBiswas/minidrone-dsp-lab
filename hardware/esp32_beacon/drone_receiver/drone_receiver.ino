// Drone ESP32: scans the anchor beacons (3 or 4), reads their RSSI, and sends
// "-62,-70,-58[,-66]" lines to the phone over Bluetooth LE (Nordic UART service)
// and to a laptop over USB serial (115200 baud). The web app's Beacon tab reads either.
#include <WiFi.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define WIFI_CHANNEL 1
#define NUM_ANCHORS 3   // set 4 if you place A4 in the far corner (and tap '4th anchor' in the app)
#define SVC_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define TX_UUID  "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLECharacteristic *tx;
bool phoneConnected = false;
int lastRssi[4] = {-100, -100, -100, -100};

class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *s) { phoneConnected = true; }
  void onDisconnect(BLEServer *s) { phoneConnected = false; s->startAdvertising(); }
};

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  BLEDevice::init("DRONE-RX");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCB());
  BLEService *svc = server->createService(SVC_UUID);
  tx = svc->createCharacteristic(TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  tx->addDescriptor(new BLE2902());
  svc->start();
  server->getAdvertising()->addServiceUUID(SVC_UUID);
  server->getAdvertising()->start();
  Serial.println("Drone receiver ready. Waiting for anchors DRONE-A1..A3");
}

void loop() {
  // scan only the anchors' channel, 80 ms per scan -> several readings per second
  int n = WiFi.scanNetworks(false, false, false, 80, WIFI_CHANNEL);
  for (int i = 0; i < n; i++) {
    String id = WiFi.SSID(i);
    if (id.startsWith("DRONE-A")) {
      int k = id.substring(7).toInt() - 1;
      if (k >= 0 && k < NUM_ANCHORS) lastRssi[k] = WiFi.RSSI(i);
    }
  }
  WiFi.scanDelete();

  char line[32];
  if (NUM_ANCHORS == 4) snprintf(line, sizeof line, "%d,%d,%d,%d\n", lastRssi[0], lastRssi[1], lastRssi[2], lastRssi[3]);
  else snprintf(line, sizeof line, "%d,%d,%d\n", lastRssi[0], lastRssi[1], lastRssi[2]);
  Serial.printf("%lu,%s", millis(), line);           // laptop / logging: time_ms,A1,A2,A3
  if (phoneConnected) {                               // phone: A1,A2,A3 (short, fits one BLE packet)
    tx->setValue((uint8_t *)line, strlen(line));
    tx->notify();
  }
}
