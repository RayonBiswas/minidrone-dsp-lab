# ESP32 beacon navigation

Three ESP32 **anchors** sit at known points in the room and broadcast Wi-Fi beacons. The ESP32 on the **drone** measures each anchor's signal strength (RSSI) several times a second and streams it to the phone app (Bluetooth LE) or the laptop (USB). The app's **Beacon** tab turns RSSI into distance, runs trilateration, filters the result and draws the drone's path from start to goal on a map.

```
A1, A2, A3 (Wi-Fi beacons, channel 1)
        │  RSSI
        ▼
Drone ESP32 ──BLE (Nordic UART)──▶ phone app (Chrome, GitHub Pages link)
            └─USB serial 115200──▶ laptop app (Chrome/Edge)
```

## Hardware (required)
- 4 × ESP32 dev board (ESP32-WROOM / DevKitC), about ₹350–450 each
- 4 × USB cable, plus USB chargers or power banks for the anchors
- No wiring needed: each board only needs USB power

## Flashing (Arduino IDE)
1. Install the **esp32 by Espressif** board package (Boards Manager), then select **ESP32 Dev Module**.
2. `anchor/anchor.ino`: flash 3 boards, setting `ANCHOR_ID` to 1, 2 and 3 in turn.
3. `drone_receiver/drone_receiver.ino`: flash the 4th board. If the sketch is too big, choose **Tools → Partition Scheme → Huge APP**.
4. Open Serial Monitor at 115200 on the drone board. You should see lines like `1234,-58,-71,-66` (time_ms, A1, A2, A3).

## Room layout (must match the app map)
| Anchor | Position (x, y) in metres |
|---|---|
| A1 | (0, 0): one corner |
| A2 | (5, 0): corner along the 5 m wall |
| A3 | (0, 4): corner along the 4 m wall |
| Start | (0.6, 0.6) |
| Goal | (4.4, 3.4), or tap the map to move it |

Put all anchors at the same height (about 1 m, on a stool or shelf) and keep the antenna side facing the room.

## Calibration (do this first)
1. Put one anchor down and hold the drone board at 0.5, 1, 2, 3, 4 and 5 m from it.
2. Log about 30 s at each distance (the Serial Monitor output, saved as CSV).
3. Fit `RSSI = P0 − 10·n·log10(d)`: P0 is the RSSI at 1 m, n is the slope. Typical indoor values are n ≈ 2–3.
4. Enter P0 and n with the Beacon tab sliders.

## Testing without hardware
- In the app: **Beacon → Simulate**. The full chain runs on simulated RSSI with shadowing noise and multipath fades.
- **Load log (CSV)** replays a recorded file. Try `sample_log.csv` (a simulated walk from start to goal), then your own logs from the Serial Monitor.

## Live
- **Phone:** power the drone ESP32 from a power bank, open the GitHub Pages link in Chrome, go to Beacon → **ESP32 via Bluetooth** and pick `DRONE-RX`.
- **Laptop:** plug the drone ESP32 in by USB, then Beacon → **ESP32 via USB** and choose the COM port.
- Walk the drone board from start to goal. Red dots are raw trilateration; the blue path is filtered.

## Upgrade: Wi-Fi FTM (optional)
ESP32-S2/S3/C3 boards support 802.11mc Fine Timing Measurement: distance from the signal's round-trip time instead of its strength, typically about 1 m accuracy and far less affected by fading. Make the anchors FTM responders and the drone the initiator, then send distances instead of RSSI.
