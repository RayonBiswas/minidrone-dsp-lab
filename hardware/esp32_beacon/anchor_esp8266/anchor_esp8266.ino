// ESP8266 anchor (NodeMCU / Wemos D1 mini / ESP-01): same beacon as the ESP32 anchor.
// Board package: "esp8266 by ESP8266 Community". Set ANCHOR_ID, flash, power from any USB charger.
#include <ESP8266WiFi.h>

#define ANCHOR_ID 2          // 1, 2, 3 or 4
#define WIFI_CHANNEL 1       // must match the receiver

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);
  WiFi.setOutputPower(8.5);   // dBm, fixed so the RSSI-distance curve stays stable
  char ssid[16];
  snprintf(ssid, sizeof ssid, "DRONE-A%d", ANCHOR_ID);
  WiFi.softAP(ssid, "dronelab123", WIFI_CHANNEL, false, 4);
  Serial.printf("Anchor %s broadcasting on channel %d\n", ssid, WIFI_CHANNEL);
}

void loop() {
  delay(1000);
}
