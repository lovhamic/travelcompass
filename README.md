# TravelCompass

**TravelCompass** is a smart, magnetometer-free GPS waypoint navigator built around the ESP32-C3 Supermini. It automatically calculates the nearest destination from a hardcoded list and provides a smooth, real-time directional arrow on a 1.3" OLED display. 

By fusing a NEO-6M GPS with an MPU6050 gyroscope using a custom 1D Kalman filter, the device solves the classic "GPS freezing" problem. The arrow updates instantly as you turn while standing still, and uses GPS Course Over Ground (COG) to correct gyroscope drift while walking.

## Features
*   **Sensor Fusion (Kalman Filter):** Combines 1Hz absolute GPS heading data with 100Hz+ relative gyroscope rotation for buttery-smooth arrow tracking without needing a magnetic compass.
*   **Auto-Targeting:** Automatically calculates distances to a database of coordinates and locks onto the nearest one upon getting a GPS fix.
*   **No Menu Clutter:** Headless operation designed for single-purpose navigation. Reach a destination, press reset, and it automatically finds the next closest point.
*   **Raw I2C Optimization:** MPU6050 data is pulled via direct I2C byte requests, skipping heavy libraries to keep the display frame rate extremely high.

## Hardware Components
*   **Microcontroller:** ESP32-C3 Supermini
*   **GPS:** NEO-6M (UART)
*   **IMU:** MPU6050 (I2C)
*   **Display:** 1.3" OLED (SH1106 driver, I2C)

## Wiring / Pinout

| ESP32-C3 Pin | Component | Pin | Notes |
| :--- | :--- | :--- | :--- |
| **3.3V** | OLED | VDD/VCC | Power for screen |
| **5V** | MPU6050 & GPS | VCC | 5V ensures onboard regulators work properly |
| **GND** | All | GND | Common ground |
| **GPIO 8** | OLED & MPU6050 | SDA | I2C Data (Shared Bus) |
| **GPIO 9** | OLED & MPU6050 | SCL | I2C Clock (Shared Bus) |
| **GPIO 20** | NEO-6M GPS | TX | UART Receive (Crossed) |
| **GPIO 21** | NEO-6M GPS | RX | UART Transmit (Crossed) |

*Note: Ensure the MPU6050 AD0 pin is floating or tied to GND so it defaults to I2C address `0x68`.*

## Software Dependencies
Install the following via the Arduino IDE Library Manager:
*   **TinyGPSPlus** by Mikal Hart (GPS NMEA parsing)
*   **Adafruit GFX Library** by Adafruit (Core graphics math)
*   **Adafruit SH110X** by Adafruit (Specific driver for the 1.3" OLED)

## How It Works (State Machine)

1.  **FIND_NEAREST:** On boot, the device waits for a valid 3D GPS fix. Once acquired, it uses the Haversine formula to calculate the distance to all locations in the `toilets` array and locks onto the closest one.
2.  **CALIBRATE:** Because there is no magnetometer, the device does not know which way True North is. The screen will prompt you to "START WALKING". Once you exceed 2.0 km/h, the GPS calculates a valid Course Over Ground (COG). The system snaps the Kalman filter's baseline to this absolute heading.
3.  **NAVIGATE:** You can now stop walking. The MPU6050 takes over, spinning the arrow relative to your hand movements. As you continue walking, the GPS continuously feeds trajectory data back into the Kalman filter to erase any gyroscope drift.
4.  **ARRIVED:** Once the calculated distance drops below 15 meters, the screen alerts you of arrival. Press the physical `RST` button on the ESP32-C3 to reboot the system and lock onto the next closest location.

## Adding Custom Locations
To change the navigation targets, simply update the `toilets` array in the main sketch. Provide the Latitude, Longitude, and a short UI Label. Update `NUM_TOILETS` to match the size of your list.

```cpp
const int NUM_TOILETS = 3;
Coord toilets[NUM_TOILETS] = {
  {11.278417, 75.844318, "Upper Washroom"},
  {11.278558, 75.844197, "Lower Washroom"},
  {11.277796, 75.844170, "Hospital IPM"}
};
```
