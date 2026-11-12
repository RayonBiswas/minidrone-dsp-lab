// ESP32 anchor: broadcasts a Wi-Fi beacon the drone ESP32 measures (RSSI).
// Flash 3 boards, changing ANCHOR_ID to 1, 2, 3. Power each from any USB charger or power bank.
#include <WiFi.h>

#define ANCHOR_ID 1          // 1, 2 or 3
#define WIFI_CHANNEL 1       // all anchors on the same channel so the drone scans fast

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);
  char ssid[16];
  snprintf(ssid, sizeof ssid, "DRONE-A%d", ANCHOR_ID);
  WiFi.softAP(ssid, "dronelab123", WIFI_CHANNEL, 0, 4);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);   // fixed transmit power keeps the RSSI-distance curve stable
  Serial.printf("Anchor %s broadcasting on channel %d\n", ssid, WIFI_CHANNEL);
}

void loop() {
  delay(1000);
}
